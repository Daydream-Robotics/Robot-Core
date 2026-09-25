//Inclusions
#include "main.h"
#include "daydream/utils/helpers.hpp"
#include "daydream/motion/odometry.hpp"
#include "daydream/mcl/mcl.hpp"
#include "daydream/mcl/rangeSensing.hpp"
#include <cmath>
#include <algorithm>

//Odometry headings already use the math frame: +x forward, +y left, CCW positive.
float RangeSensing::fieldHeadingToMathHeading(float fieldHeadingRad) {
    return fieldHeadingRad;
}

//initializes lookup tables and precomputed data
//call this ONCE at startup before using MCL
void RangeSensing::initRangeSystem() {
    //build sin/cos lookup tables (0.1 degree resolution)
    for (int i = 0; i < ANGLE_LUT_SIZE; i++) {
        float angle = (i * 2.0f * M_PI) / ANGLE_LUT_SIZE;
        m_sinLut[i] = std::sin(angle);
        m_cosLut[i] = std::cos(angle);
    }

    //precompute segment properties (walls + obstacles)
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        const LineSegment& seg = LINE_SEGMENTS[i];
        m_precomputedSegments[i].x1 = seg.x1;
        m_precomputedSegments[i].y1 = seg.y1;
        m_precomputedSegments[i].dx = seg.x2 - seg.x1;
        m_precomputedSegments[i].dy = seg.y2 - seg.y1;
    }
}

//fast sin/cos lookup using precomputed table
void RangeSensing::fastSincos(float theta, float& sinOut, float& cosOut) {
    //normalize angle to [0, 2π)
    float normalized = theta;
    while (normalized < 0.0f) normalized += 2.0f * M_PI;
    while (normalized >= 2.0f * M_PI) normalized -= 2.0f * M_PI;

    //convert radians to lookup table index
    int index = (int)((normalized * ANGLE_LUT_SIZE) / (2.0f * M_PI));

    //clamp to valid range (safety check)
    if (index >= ANGLE_LUT_SIZE) index = ANGLE_LUT_SIZE - 1;
    if (index < 0) index = 0;

    sinOut = m_sinLut[index];
    cosOut = m_cosLut[index];
}

//performs raycasting from particle position to find expected sensor readings
RangeReadings RangeSensing::raycast(const mcl::Particle& particle) {
    RangeReadings expectedReadings;

    //Use the odometry heading directly for rotation math.
    float headingSin, headingCos;
    fastSincos(fieldHeadingToMathHeading(static_cast<float>(particle.theta)), headingSin, headingCos);

    //raycast for each of the 4 distance sensors
    for (int sensorDir = 0; sensorDir < 4; sensorDir++) {
        //calculate sensor position in global frame
        //local frame uses +x = forward, +y = left
        const float sensorGlobalX = particle.x +
            (SENSOR_OFFSET_X[sensorDir] * headingCos + SENSOR_OFFSET_Y[sensorDir] * headingSin);
        const float sensorGlobalY = particle.y +
            (-SENSOR_OFFSET_X[sensorDir] * headingSin + SENSOR_OFFSET_Y[sensorDir] * headingCos);

        //calculate sensor direction in global frame
        //sensor basis uses +x = forward, +y = left
        const float cosSensor =
            SIN_HEADINGS[sensorDir] * headingCos + COS_HEADINGS[sensorDir] * headingSin;
        const float sinSensor =
            -SIN_HEADINGS[sensorDir] * headingSin + COS_HEADINGS[sensorDir] * headingCos;

        float minDistance = INFINITY;

        //test ray against all wall segments
        for (int segmentIndex = 0; segmentIndex < NUM_SEGMENTS; segmentIndex++) {
            const PrecomputedSegment& segment = m_precomputedSegments[segmentIndex];

            const float xOffset = segment.x1 - sensorGlobalX;
            const float yOffset = segment.y1 - sensorGlobalY;

            //calculate denominator for ray-line intersection
            const float denominator = cosSensor * segment.dy - sinSensor * segment.dx;

            //skip if ray is parallel to segment
            if (denominator > -1e-9f && denominator < 1e-9f) {
                continue;
            }

            const float invDenom = 1.0f / denominator;

            //calculate intersection parameters
            //t = distance along ray, u = position along segment
            const float t = (xOffset * segment.dy - yOffset * segment.dx) * invDenom;
            const float u = (xOffset * sinSensor - yOffset * cosSensor) * invDenom;

            //check if intersection is valid and in front of sensor
            if (t >= 0.0f && u >= 0.0f && u <= 1.0f) {
                //update minimum distance
                if (t < minDistance) {
                    minDistance = t;
                }
            }
        }

        //store result (NAN if no intersection found)
        expectedReadings.direction[sensorDir] = (minDistance == INFINITY) ? NAN : minDistance;
    }

    return expectedReadings;
}

//optimized raycast for when all particles have same theta
//precomputes sin/cos and sensor directions once, then only varies position
RangeReadings RangeSensing::raycastFixedTheta(const mcl::Particle& particle,
                                              float headingSin, float headingCos,
                                              const float cosSensors[4], const float sinSensors[4]) {
    RangeReadings expectedReadings;

    //raycast for each of the 4 distance sensors
    for (int sensorDir = 0; sensorDir < 4; sensorDir++) {
        //calculate sensor position in global frame
        const float sensorGlobalX = particle.x +
            (SENSOR_OFFSET_X[sensorDir] * headingCos + SENSOR_OFFSET_Y[sensorDir] * headingSin);
        const float sensorGlobalY = particle.y +
            (-SENSOR_OFFSET_X[sensorDir] * headingSin + SENSOR_OFFSET_Y[sensorDir] * headingCos);

        //use precomputed sensor directions
        const float cosSensor = cosSensors[sensorDir];
        const float sinSensor = sinSensors[sensorDir];

        float minDistance = INFINITY;

        //test ray against all wall segments
        for (int segmentIndex = 0; segmentIndex < NUM_SEGMENTS; segmentIndex++) {
            const PrecomputedSegment& segment = m_precomputedSegments[segmentIndex];

            const float xOffset = segment.x1 - sensorGlobalX;
            const float yOffset = segment.y1 - sensorGlobalY;

            const float denominator = cosSensor * segment.dy - sinSensor * segment.dx;

            //skip if ray is parallel to segment
            if (denominator > -1e-9f && denominator < 1e-9f) {
                continue;
            }

            const float invDenom = 1.0f / denominator;
            const float t = (xOffset * segment.dy - yOffset * segment.dx) * invDenom;
            const float u = (xOffset * sinSensor - yOffset * cosSensor) * invDenom;

            //check if intersection is valid and in front of sensor
            if (t >= 0.0f && u >= 0.0f && u <= 1.0f) {
                if (t < minDistance) {
                    minDistance = t;
                }
            }
        }

        expectedReadings.direction[sensorDir] = (minDistance == INFINITY) ? NAN : minDistance;
    }

    return expectedReadings;
}

//prints estimated position from sensor readings
void RangeSensing::printPositionFromSensors() {
    //update sensor readings first to get fresh data
    updateRangeSensors();

    RangeReadings readings = distanceReadings;

    double x = 0.0;
    double y = 0.0;
    int validCount = 0;

    //calculate position from each sensor
    //front sensor points in +x direction, measures distance to right wall
    if (!std::isnan(readings.front)) {
        x = X_MAX - readings.front + SENSOR_OFFSET_X[0];
        validCount++;
    }

    //back sensor points in -x direction, measures distance to left wall
    if (!std::isnan(readings.back)) {
        x = X_MIN + readings.back + SENSOR_OFFSET_X[2];
        validCount++;
    }

    //left sensor points in +y direction, measures distance to top wall
    if (!std::isnan(readings.left)) {
        y = Y_MAX - readings.left + SENSOR_OFFSET_Y[1];
        validCount++;
    }

    //right sensor points in -y direction, measures distance to bottom wall
    if (!std::isnan(readings.right)) {
        y = Y_MIN + readings.right + SENSOR_OFFSET_Y[3];
        validCount++;
    }

    //print results
    if (validCount > 0) {
        printf("Estimated Position from Sensors (field center = 0,0):\n");
        printf("  x: %.2f\" (forward/back, +x = forward)\n", x);
        printf("  y: %.2f\" (left/right, +y = left)\n", y);
        printf("  Valid sensors: %d/4\n", validCount);
        printf("  Readings - F:%.1f\" L:%.1f\" B:%.1f\" R:%.1f\"\n",
               readings.front, readings.left, readings.back, readings.right);
    } else {
        printf("No valid sensor readings available\n");
    }
}

//calculates robot position using 1 or 2 sensors + IMU heading
RangeSensing::Position2d RangeSensing::calculatePositionFrom2Sensors(int sensor1Idx, int sensor2Idx, bool silent) {
    //Get the current pose from odometry.
    Pose currentPose = odom.getPose();

    if (!silent) {
        printf("Current Pose: X: %.6f, Y: %.6f, Theta: %.6f\n",
               currentPose.x, currentPose.y, currentPose.theta);
    }

    //initialize result with current pose
    Position2d result = {
        currentPose.x,
        currentPose.y,
        currentPose.theta,
        false
    };

    //convert robot heading to math heading
    double headingRad = currentPose.theta;
    double c = std::cos(headingRad);
    double s = std::sin(headingRad);

    //update sensors and get readings
    updateRangeSensors();
    RangeReadings readings = distanceReadings;
    double sensorVals[4] = {
        readings.front,
        readings.left,
        readings.back,
        readings.right
    };

    //lambda function to process each sensor
    auto applySensor = [&](int idx) {
        if (idx < 0 || idx > 3) return;
        double dist = sensorVals[idx];
        if (std::isnan(dist)) return;

        //rotate sensor offset to field frame
        double offX = SENSOR_OFFSET_X[idx] * c + SENSOR_OFFSET_Y[idx] * s;
        double offY = -SENSOR_OFFSET_X[idx] * s + SENSOR_OFFSET_Y[idx] * c;

        //calculate sensor position in field frame
        double sx = currentPose.x + offX;
        double sy = currentPose.y + offY;

        //rotate sensor direction to field frame
        double dirXRobot = SIN_HEADINGS[idx];  //forward component
        double dirYRobot = COS_HEADINGS[idx];  //left component

        double dx = dirXRobot * c + dirYRobot * s;
        double dy = -dirXRobot * s + dirYRobot * c;

        if (!silent) {
            printf("Sensor %d offset: local(%.2f, %.2f) -> global(%.2f, %.2f)\n",
                   idx, SENSOR_OFFSET_X[idx], SENSOR_OFFSET_Y[idx], offX, offY);
            printf("Sensor %d pos: (%.2f, %.2f), direction: dx=%.2f, dy=%.2f, dist=%.2f\n",
                   idx, sx, sy, dx, dy, dist);
        }

        //calculate distances to walls
        double tX = INFINITY;
        double tY = INFINITY;
        int whichXWall = 0;
        int whichYWall = 0;

        //calculate distance to X walls
        if (std::abs(dx) > 1e-6) {
            if (dx > 0) {
                tX = (X_MAX - sx) / dx;
                whichXWall = 1;
            } else {
                tX = (X_MIN - sx) / dx;
                whichXWall = -1;
            }
        }

        //calculate distance to Y walls
        if (std::abs(dy) > 1e-6) {
            if (dy > 0) {
                tY = (Y_MAX - sy) / dy;
                whichYWall = 1;
            } else {
                tY = (Y_MIN - sy) / dy;
                whichYWall = -1;
            }
        }

        if (!silent) {
            printf("  Wall distances: t_x=%.2f, t_y=%.2f, sensor_dist=%.2f\n", tX, tY, dist);
        }

        //check which wall is hit based on discrepancy
        double xDiscrepancy = std::abs(tX - dist);
        double yDiscrepancy = std::abs(tY - dist);

        const double tolerance = 10.0;
        bool hitXWall = (tX > 0) && (tX < tY) && (xDiscrepancy < tolerance);
        bool hitYWall = (tY > 0) && (tY < tX) && (yDiscrepancy < tolerance);

        //update result based on which wall was hit
        if (hitXWall) {
            double wallX = (whichXWall > 0) ? X_MAX : X_MIN;
            result.x = wallX - offX - dx * dist;
            if (!silent) {
                printf("Sensor %d updated X: %.2f (hit %s wall, disc=%.2f)\n",
                       idx, result.x, (whichXWall > 0) ? "X_MAX" : "X_MIN", xDiscrepancy);
            }
            result.valid = true;
        } else if (hitYWall) {
            double wallY = (whichYWall > 0) ? Y_MAX : Y_MIN;
            result.y = wallY - offY - dy * dist;
            if (!silent) {
                printf("Sensor %d updated Y: %.2f (hit %s wall, disc=%.2f)\n",
                       idx, result.y, (whichYWall > 0) ? "Y_MAX" : "Y_MIN", yDiscrepancy);
            }
            result.valid = true;
        } else {
            if (!silent) {
                printf("Sensor %d: discrepancy (X: %.2f, Y: %.2f)\n", idx, xDiscrepancy, yDiscrepancy);
            }
        }
    };

    //apply both sensors
    applySensor(sensor1Idx);
    applySensor(sensor2Idx);

    if (!silent) {
        printf("Result Pose: X: %.6f, Y: %.6f, Theta: %.6f\n\n",
               result.x, result.y, result.theta);
    }

    return result;
}


//checks raycast accuracy at current position
void RangeSensing::checkRaycastAccuracy() {
    printf("=== RAYCAST ACCURACY CHECK ===\n");

    //get current odometry position
    const Pose currentPos = odom.getPose();

    //get actual sensor readings
    updateRangeSensors();
    RangeReadings actual = distanceReadings;

    //create a particle at the current odometry position
    mcl::Particle testParticle;
    testParticle.x = currentPos.x;
    testParticle.y = currentPos.y;
    testParticle.theta = currentPos.theta;
    testParticle.weight = 1.0;
    testParticle.normalizedWeight = 1.0;
    testParticle.logWeight = 0.0;

    //raycast from this position
    RangeReadings expected = raycast(testParticle);

    //print comparison
    printf("\nCurrent Odometry Position:\n");
    printf("  X:     %7.2f\"\n", currentPos.x);
    printf("  Y:     %7.2f\"\n", currentPos.y);
    printf("  Theta: %7.2f° (%7.4f rad)\n", currentPos.theta * (180.0 / M_PI), currentPos.theta);

    printf("\nSensor Readings vs Raycast Predictions:\n");
    printf("  Sensor    | Actual  | Expected | Error   | Status\n");
    printf("  ----------|---------|----------|---------|--------\n");

    //lambda to check each sensor
    auto checkSensor = [](const char* name, double actual, double expected) {
        if (std::isnan(actual) || std::isnan(expected)) {
            printf("  %-9s | %7s | %8s | %7s | INVALID\n",
                   name,
                   std::isnan(actual) ? "NaN" : "OK",
                   std::isnan(expected) ? "NaN" : "OK",
                   "---");
            return;
        }
        double error = actual - expected;
        const char* status = (fabs(error) < 2.0) ? "GOOD" :
                            (fabs(error) < 5.0) ? "OK" : "BAD";
        printf("  %-9s | %7.2f | %8.2f | %+7.2f | %s\n",
               name, actual, expected, error, status);
    };

    checkSensor("Front", actual.front, expected.front);
    checkSensor("Left", actual.left, expected.left);
    checkSensor("Back", actual.back, expected.back);
    checkSensor("Right", actual.right, expected.right);

    //calculate overall error
    double totalError = 0.0;
    int validCount = 0;

    if (!std::isnan(actual.front) && !std::isnan(expected.front)) {
        totalError += fabs(actual.front - expected.front);
        validCount++;
    }
    if (!std::isnan(actual.left) && !std::isnan(expected.left)) {
        totalError += fabs(actual.left - expected.left);
        validCount++;
    }
    if (!std::isnan(actual.back) && !std::isnan(expected.back)) {
        totalError += fabs(actual.back - expected.back);
        validCount++;
    }
    if (!std::isnan(actual.right) && !std::isnan(expected.right)) {
        totalError += fabs(actual.right - expected.right);
        validCount++;
    }

    //print average error and status
    if (validCount > 0) {
        double avgError = totalError / validCount;
        printf("\nAverage Error: %.2f\" (%d/%d sensors valid)\n", avgError, validCount, 4);

        if (avgError < 2.0) {
            printf("Status: ✓ EXCELLENT - Odometry matches sensors well\n");
        } else if (avgError < 5.0) {
            printf("Status: ⚠ OK - Some drift, but acceptable\n");
        } else {
            printf("Status: ✗ POOR - Large error, check odometry or sensor calibration\n");
        }
    } else {
        printf("\nNo valid sensors to compare!\n");
    }

    printf("==============================\n");
}

//prints sensor calibration factors for known distance
void RangeSensing::printSensorCalibration(double knownDistance) {
    //update sensor readings first to get fresh data
    updateRangeSensors();

    RangeReadings readings = distanceReadings;

    printf("Sensor Calibration (known distance: %.1f\"):\n", knownDistance);
    printf("Raw sensor values (mm):\n");
    printf("  Front raw: %ld mm\n", distanceSensors.front.get_distance());
    printf("  Left raw:  %ld mm\n", distanceSensors.left.get_distance());
    printf("  Back raw:  %ld mm\n", distanceSensors.back.get_distance());
    printf("  Right raw: %ld mm\n", distanceSensors.right.get_distance());
    printf("\n");

    //calculate and print calibration scale for each sensor
    if (!std::isnan(readings.front) && readings.front > 0.1) {
        printf("  Front: %.2f\" -> scale = %.4f\n",
               readings.front, knownDistance / readings.front);
    } else {
        printf("  Front: sensor error or not connected\n");
    }
    if (!std::isnan(readings.left) && readings.left > 0.1) {
        printf("  Left:  %.2f\" -> scale = %.4f\n",
               readings.left, knownDistance / readings.left);
    } else {
        printf("  Left: sensor error or not connected\n");
    }
    if (!std::isnan(readings.back) && readings.back > 0.1) {
        printf("  Back:  %.2f\" -> scale = %.4f\n",
               readings.back, knownDistance / readings.back);
    } else {
        printf("  Back: sensor error or not connected\n");
    }
    if (!std::isnan(readings.right) && readings.right > 0.1) {
        printf("  Right: %.2f\" -> scale = %.4f\n",
               readings.right, knownDistance / readings.right);
    } else {
        printf("  Right: sensor error or not connected\n");
    }
}

//measures and prints range sensor noise statistics
void RangeSensing::measureRangeNoise(int numSamples) {
    printf("Measuring range sensor noise (%d samples)...\n", numSamples);
    printf("Keep robot stationary!\n\n");

    //accumulators for statistics
    double sum[4] = {0, 0, 0, 0};
    double sumSq[4] = {0, 0, 0, 0};
    int count[4] = {0, 0, 0, 0};

    //collect samples
    for (int i = 0; i < numSamples; i++) {
        updateRangeSensors();
        RangeReadings readings = distanceReadings;

        if (!std::isnan(readings.front)) {
            sum[0] += readings.front;
            sumSq[0] += readings.front * readings.front;
            count[0]++;
        }
        if (!std::isnan(readings.left)) {
            sum[1] += readings.left;
            sumSq[1] += readings.left * readings.left;
            count[1]++;
        }
        if (!std::isnan(readings.back)) {
            sum[2] += readings.back;
            sumSq[2] += readings.back * readings.back;
            count[2]++;
        }
        if (!std::isnan(readings.right)) {
            sum[3] += readings.right;
            sumSq[3] += readings.right * readings.right;
            count[3]++;
        }

        pros::delay(10);
    }

    //calculate and print statistics
    printf("Range Sensor Noise Statistics:\n");
    const char* names[4] = {"Front", "Left", "Back", "Right"};

    for (int i = 0; i < 4; i++) {
        if (count[i] > 0) {
            double mean = sum[i] / count[i];
            double variance = (sumSq[i] / count[i]) - (mean * mean);
            double stddev = sqrt(variance);

            printf("  %s: mean=%.2f\", stddev=%.3f\" (use %.3f for sigma)\n",
                   names[i], mean, stddev, stddev);
        } else {
            printf("  %s: no valid readings\n", names[i]);
        }
    }
}
