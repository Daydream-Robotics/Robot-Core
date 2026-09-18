//Inclusions
#include "main.h"
#include "control/sensors.hpp"
#include "control/odom.hpp"
#include "control/mcl.hpp"
#include "control/range.hpp"
#include <cmath>
#include <algorithm>

//Created namespace range
namespace range {
    //current odometry position
    odom::odom_position current_odom_pos;

    inline float field_heading_to_math_heading(float field_heading_rad) {
        return field_heading_rad - static_cast<float>(M_PI_2);
    }

    //sensor direction offsets relative to robot heading
    //coordinate system: 0°=left, 90°=up, 180°=right, 270°=down
    //front sensor: 0° offset from robot heading
    //left sensor: +90° offset (CCW) from robot heading
    //back sensor: +180° offset from robot heading
    //right sensor: +270° offset (or -90°) from robot heading
    const int sin_headings[4] = {1, 0, -1, 0};   // front(0°), left(90°), back(180°), right(270°)
    const int cos_headings[4] = {0, 1, 0, -1};   // front(0°), left(90°), back(180°), right(270°)

    //sensor physical offsets from robot center in inches
    const float sensor_offset_x[4] = {-2.8, -5, -4.0, -5};      //front, left, back, right
    const float sensor_offset_y[4] = {5.0, 4.75, -5.0, -4.75};  //front, left, back, right

    //field boundaries in inches
    constexpr double X_MIN = -70.0;
    constexpr double X_MAX = 70.0;
    constexpr double Y_MIN = -70.0;
    constexpr double Y_MAX = 70.0;

    //matchloader dimensions in inches
    constexpr double MATCHLOADER_DIAMETER = 4.17;
    constexpr double MATCHLOADER_RADIUS = MATCHLOADER_DIAMETER/2;

    //matchloader center x positions in inches
    constexpr double ML_X_LEFT  = -46.6;
    constexpr double ML_X_RIGHT =  46.6;

    //matchloader edge x positions
    constexpr double XL_L = ML_X_LEFT  - MATCHLOADER_RADIUS;  // -48.685
    constexpr double XR_L = ML_X_LEFT  + MATCHLOADER_RADIUS;  // -44.515
    constexpr double XL_R = ML_X_RIGHT - MATCHLOADER_RADIUS;  //  44.515
    constexpr double XR_R = ML_X_RIGHT + MATCHLOADER_RADIUS;  //  48.685

    //matchloader edge y positions
    constexpr double Y_TOP_BOTTOM =  70.0 - MATCHLOADER_DIAMETER;   //  65.83
    constexpr double Y_BOT_TOP    = -70.0 + MATCHLOADER_DIAMETER;   // -65.83

    //field boundary walls (x1, x2, y1, y2) - field is 140" × 140"
    const line_segment wall_left   = {-70, -70, -70, 70};   // left wall (x = -70)
    const line_segment wall_right  = {70, 70, -70, 70};     // right wall (x = +70)
    const line_segment wall_top    = {-70, 70, 70, 70};     // top wall (y = +70)
    const line_segment wall_bottom = {-70, 70, -70, -70};   // bottom wall (y = -70)

    //matchloader obstacle segments - top left
    const line_segment matchloader_top_left_left   = {XL_L, XL_L, Y_TOP_BOTTOM, 70};
    const line_segment matchloader_top_left_right  = {XR_L, XR_L, Y_TOP_BOTTOM, 70};
    const line_segment matchloader_top_left_bottom = {XL_L, XR_L, Y_TOP_BOTTOM, Y_TOP_BOTTOM};

    //matchloader obstacle segments - top right
    const line_segment matchloader_top_right_left   = {XL_R, XL_R, Y_TOP_BOTTOM, 70};
    const line_segment matchloader_top_right_right  = {XR_R, XR_R, Y_TOP_BOTTOM, 70};
    const line_segment matchloader_top_right_bottom = {XL_R, XR_R, Y_TOP_BOTTOM, Y_TOP_BOTTOM};

    //matchloader obstacle segments - bottom left
    const line_segment matchloader_bottom_left_left   = {XL_L, XL_L, -70, Y_BOT_TOP};
    const line_segment matchloader_bottom_left_right  = {XR_L, XR_L, -70, Y_BOT_TOP};
    const line_segment matchloader_bottom_left_top    = {XL_L, XR_L, Y_BOT_TOP, Y_BOT_TOP};

    //matchloader obstacle segments - bottom right
    const line_segment matchloader_bottom_right_left   = {XL_R, XL_R, -70, Y_BOT_TOP};
    const line_segment matchloader_bottom_right_right  = {XR_R, XR_R, -70, Y_BOT_TOP};
    const line_segment matchloader_bottom_right_top    = {XL_R, XR_R, Y_BOT_TOP, Y_BOT_TOP};

    //all line segments to test for raycasting
    const line_segment line_segments[] = {
        wall_left,
        wall_right,
        wall_top,
        wall_bottom,

        matchloader_top_left_left,
        matchloader_top_left_right,
        matchloader_top_left_bottom,

        matchloader_top_right_left,
        matchloader_top_right_right,
        matchloader_top_right_bottom,

        matchloader_bottom_left_left,
        matchloader_bottom_left_right,
        matchloader_bottom_left_top,

        matchloader_bottom_right_left,
        matchloader_bottom_right_right,
        matchloader_bottom_right_top
    };

    //number of line segments for raycasting
    constexpr int NUM_SEGMENTS = sizeof(line_segments) / sizeof(line_segment);

    //precomputed segment properties (initialized in init_range_system)
    precomputed_segment precomputed_segments[NUM_SEGMENTS];

    //sin/cos lookup tables for fast trig calculations
    float sin_lut[ANGLE_LUT_SIZE];
    float cos_lut[ANGLE_LUT_SIZE];

    //initializes lookup tables and precomputed data
    //call this ONCE at startup before using MCL
    void init_range_system() {
        //build sin/cos lookup tables (0.1 degree resolution)
        for (int i = 0; i < ANGLE_LUT_SIZE; i++) {
            float angle = (i * 2.0f * M_PI) / ANGLE_LUT_SIZE;
            sin_lut[i] = std::sin(angle);
            cos_lut[i] = std::cos(angle);
        }

        //precompute segment properties (walls + obstacles)
        for (int i = 0; i < NUM_SEGMENTS; i++) {
            const line_segment& seg = line_segments[i];
            precomputed_segments[i].x1 = seg.x_1;
            precomputed_segments[i].y1 = seg.y_1;
            precomputed_segments[i].dx = seg.x_2 - seg.x_1;
            precomputed_segments[i].dy = seg.y_2 - seg.y_1;
        }
    }

    //fast sin/cos lookup using precomputed table
    void fast_sincos(float theta, float& sin_out, float& cos_out) {
        //normalize angle to [0, 2π)
        float normalized = theta;
        while (normalized < 0.0f) normalized += 2.0f * M_PI;
        while (normalized >= 2.0f * M_PI) normalized -= 2.0f * M_PI;

        //convert radians to lookup table index
        int index = (int)((normalized * ANGLE_LUT_SIZE) / (2.0f * M_PI));

        //clamp to valid range (safety check)
        if (index >= ANGLE_LUT_SIZE) index = ANGLE_LUT_SIZE - 1;
        if (index < 0) index = 0;

        sin_out = sin_lut[index];
        cos_out = cos_lut[index];
    }

    //performs raycasting from particle position to find expected sensor readings
    sensors::range_readings raycast(const mcl::Particle& particle) {
        sensors::range_readings distance_readings;

        //convert chassis/field heading (90 deg = +x) to math heading for rotation math
        float heading_sin, heading_cos;
        fast_sincos(field_heading_to_math_heading(static_cast<float>(particle.theta)), heading_sin, heading_cos);

        //raycast for each of the 4 distance sensors
        for (int sensor_dir = 0; sensor_dir < 4; sensor_dir++) {
            //calculate sensor position in global frame
            //local frame uses +x = forward, +y = left
            const float sensor_global_x = particle.x +
                (sensor_offset_x[sensor_dir] * heading_cos + sensor_offset_y[sensor_dir] * heading_sin);
            const float sensor_global_y = particle.y +
                (-sensor_offset_x[sensor_dir] * heading_sin + sensor_offset_y[sensor_dir] * heading_cos);

            //calculate sensor direction in global frame
            //sensor basis uses +x = forward, +y = left
            const float cos_sensor =
                sin_headings[sensor_dir] * heading_cos + cos_headings[sensor_dir] * heading_sin;
            const float sin_sensor =
                -sin_headings[sensor_dir] * heading_sin + cos_headings[sensor_dir] * heading_cos;

            float min_distance = INFINITY;

            //test ray against all wall segments
            for (int segment_index = 0; segment_index < NUM_SEGMENTS; segment_index++) {
                const precomputed_segment& segment = precomputed_segments[segment_index];

                const float x_offset = segment.x1 - sensor_global_x;
                const float y_offset = segment.y1 - sensor_global_y;

                //calculate denominator for ray-line intersection
                const float denominator = cos_sensor * segment.dy - sin_sensor * segment.dx;

                //skip if ray is parallel to segment
                if (denominator > -1e-9f && denominator < 1e-9f) {
                    continue;
                }

                const float inv_denom = 1.0f / denominator;

                //calculate intersection parameters
                //t = distance along ray, u = position along segment
                const float t = (x_offset * segment.dy - y_offset * segment.dx) * inv_denom;
                const float u = (x_offset * sin_sensor - y_offset * cos_sensor) * inv_denom;

                //check if intersection is valid and in front of sensor
                if (t >= 0.0f && u >= 0.0f && u <= 1.0f) {
                    //update minimum distance
                    if (t < min_distance) {
                        min_distance = t;
                    }
                }
            }

            //store result (NAN if no intersection found)
            distance_readings.direction[sensor_dir] = (min_distance == INFINITY) ? NAN : min_distance;
        }

        return distance_readings;
    }

    //optimized raycast for when all particles have same theta
    //precomputes sin/cos and sensor directions once, then only varies position
    sensors::range_readings raycast_fixed_theta(const mcl::Particle& particle,
                                                 float heading_sin, float heading_cos,
                                                 const float cos_sensors[4], const float sin_sensors[4]) {
        sensors::range_readings distance_readings;

        //raycast for each of the 4 distance sensors
        for (int sensor_dir = 0; sensor_dir < 4; sensor_dir++) {
            //calculate sensor position in global frame
            const float sensor_global_x = particle.x +
                (sensor_offset_x[sensor_dir] * heading_cos + sensor_offset_y[sensor_dir] * heading_sin);
            const float sensor_global_y = particle.y +
                (-sensor_offset_x[sensor_dir] * heading_sin + sensor_offset_y[sensor_dir] * heading_cos);

            //use precomputed sensor directions
            const float cos_sensor = cos_sensors[sensor_dir];
            const float sin_sensor = sin_sensors[sensor_dir];

            float min_distance = INFINITY;

            //test ray against all wall segments
            for (int segment_index = 0; segment_index < NUM_SEGMENTS; segment_index++) {
                const precomputed_segment& segment = precomputed_segments[segment_index];

                const float x_offset = segment.x1 - sensor_global_x;
                const float y_offset = segment.y1 - sensor_global_y;

                const float denominator = cos_sensor * segment.dy - sin_sensor * segment.dx;

                //skip if ray is parallel to segment
                if (denominator > -1e-9f && denominator < 1e-9f) {
                    continue;
                }

                const float inv_denom = 1.0f / denominator;
                const float t = (x_offset * segment.dy - y_offset * segment.dx) * inv_denom;
                const float u = (x_offset * sin_sensor - y_offset * cos_sensor) * inv_denom;

                //check if intersection is valid and in front of sensor
                if (t >= 0.0f && u >= 0.0f && u <= 1.0f) {
                    if (t < min_distance) {
                        min_distance = t;
                    }
                }
            }

            distance_readings.direction[sensor_dir] = (min_distance == INFINITY) ? NAN : min_distance;
        }

        return distance_readings;
    }

    //prints estimated position from sensor readings
    void print_position_from_sensors() {
        //update sensor readings first to get fresh data
        sensors::update_range_sensors();

        sensors::range_readings readings = sensors::distance_readings;

        double x = 0.0;
        double y = 0.0;
        int valid_count = 0;

        //calculate position from each sensor
        //front sensor points in +x direction, measures distance to right wall
        if (!std::isnan(readings.front)) {
            x = X_MAX - readings.front + sensor_offset_x[0];
            valid_count++;
        }

        //back sensor points in -x direction, measures distance to left wall
        if (!std::isnan(readings.back)) {
            x = X_MIN + readings.back + sensor_offset_x[2];
            valid_count++;
        }

        //left sensor points in +y direction, measures distance to top wall
        if (!std::isnan(readings.left)) {
            y = Y_MAX - readings.left + sensor_offset_y[1];
            valid_count++;
        }

        //right sensor points in -y direction, measures distance to bottom wall
        if (!std::isnan(readings.right)) {
            y = Y_MIN + readings.right + sensor_offset_y[3];
            valid_count++;
        }

        //print results
        if (valid_count > 0) {
            printf("Estimated Position from Sensors (field center = 0,0):\n");
            printf("  x: %.2f\" (forward/back, +x = forward)\n", x);
            printf("  y: %.2f\" (left/right, +y = left)\n", y);
            printf("  Valid sensors: %d/4\n", valid_count);
            printf("  Readings - F:%.1f\" L:%.1f\" B:%.1f\" R:%.1f\"\n",
                   readings.front, readings.left, readings.back, readings.right);
        } else {
            printf("No valid sensor readings available\n");
        }
    }

//calculates robot position using 1 or 2 sensors + IMU heading
position_2d calculate_position_from_2_sensors(int sensor1_idx, int sensor2_idx, bool silent) {
    //get current pose from chassis
    lemlib::Pose current_pose = drive::chassis.getPose();

    if (!silent) {
        printf("Current Pose: X: %.6f, Y: %.6f, Theta: %.6f\n",
               current_pose.x, current_pose.y, current_pose.theta);
    }

    //initialize result with current pose
    position_2d result = {
        current_pose.x,
        current_pose.y,
        current_pose.theta,
        false
    };

    //convert robot heading to math heading
    double heading_rad = (current_pose.theta - 90.0) * M_PI / 180.0;
    double c = std::cos(heading_rad);
    double s = std::sin(heading_rad);

    //update sensors and get readings
    sensors::update_range_sensors();
    sensors::range_readings readings = sensors::distance_readings;
    double sensor_vals[4] = {
        readings.front,
        readings.left,
        readings.back,
        readings.right
    };

    //lambda function to process each sensor
    auto apply_sensor = [&](int idx) {
        if (idx < 0 || idx > 3) return;
        double dist = sensor_vals[idx];
        if (std::isnan(dist)) return;

        //rotate sensor offset to field frame
        double off_x = sensor_offset_x[idx] * c + sensor_offset_y[idx] * s;
        double off_y = -sensor_offset_x[idx] * s + sensor_offset_y[idx] * c;

        //calculate sensor position in field frame
        double sx = current_pose.x + off_x;
        double sy = current_pose.y + off_y;

        //rotate sensor direction to field frame
        double dir_x_robot = sin_headings[idx];  //forward component
        double dir_y_robot = cos_headings[idx];  //left component

        double dx = dir_x_robot * c + dir_y_robot * s;
        double dy = -dir_x_robot * s + dir_y_robot * c;

        if (!silent) {
            printf("Sensor %d offset: local(%.2f, %.2f) -> global(%.2f, %.2f)\n",
                   idx, sensor_offset_x[idx], sensor_offset_y[idx], off_x, off_y);
            printf("Sensor %d pos: (%.2f, %.2f), direction: dx=%.2f, dy=%.2f, dist=%.2f\n",
                   idx, sx, sy, dx, dy, dist);
        }

        //calculate distances to walls
        double t_x = INFINITY;
        double t_y = INFINITY;
        int which_x_wall = 0;
        int which_y_wall = 0;

        //calculate distance to X walls
        if (std::abs(dx) > 1e-6) {
            if (dx > 0) {
                t_x = (X_MAX - sx) / dx;
                which_x_wall = 1;
            } else {
                t_x = (X_MIN - sx) / dx;
                which_x_wall = -1;
            }
        }

        //calculate distance to Y walls
        if (std::abs(dy) > 1e-6) {
            if (dy > 0) {
                t_y = (Y_MAX - sy) / dy;
                which_y_wall = 1;
            } else {
                t_y = (Y_MIN - sy) / dy;
                which_y_wall = -1;
            }
        }

        if (!silent) {
            printf("  Wall distances: t_x=%.2f, t_y=%.2f, sensor_dist=%.2f\n", t_x, t_y, dist);
        }

        //check which wall is hit based on discrepancy
        double x_discrepancy = std::abs(t_x - dist);
        double y_discrepancy = std::abs(t_y - dist);

        const double tolerance = 10.0;
        bool hit_x_wall = (t_x > 0) && (t_x < t_y) && (x_discrepancy < tolerance);
        bool hit_y_wall = (t_y > 0) && (t_y < t_x) && (y_discrepancy < tolerance);

        //update result based on which wall was hit
        if (hit_x_wall) {
            double wall_x = (which_x_wall > 0) ? X_MAX : X_MIN;
            result.x = wall_x - off_x - dx * dist;
            if (!silent) {
                printf("Sensor %d updated X: %.2f (hit %s wall, disc=%.2f)\n",
                       idx, result.x, (which_x_wall > 0) ? "X_MAX" : "X_MIN", x_discrepancy);
            }
            result.valid = true;
        } else if (hit_y_wall) {
            double wall_y = (which_y_wall > 0) ? Y_MAX : Y_MIN;
            result.y = wall_y - off_y - dy * dist;
            if (!silent) {
                printf("Sensor %d updated Y: %.2f (hit %s wall, disc=%.2f)\n",
                       idx, result.y, (which_y_wall > 0) ? "Y_MAX" : "Y_MIN", y_discrepancy);
            }
            result.valid = true;
        } else {
            if (!silent) {
                printf("Sensor %d: discrepancy (X: %.2f, Y: %.2f)\n", idx, x_discrepancy, y_discrepancy);
            }
        }
    };

    //apply both sensors
    apply_sensor(sensor1_idx);
    apply_sensor(sensor2_idx);

    if (!silent) {
        printf("Result Pose: X: %.6f, Y: %.6f, Theta: %.6f\n\n",
               result.x, result.y, result.theta);
    }

    return result;
}


    //checks raycast accuracy at current position
    void check_raycast_accuracy() {
        printf("=== RAYCAST ACCURACY CHECK ===\n");

        //get current odometry position
        odom::odom_position current_pos;
        {
            odom::odom_update_mutex.lock();
            current_pos = odom::chassis_odom_position;
            odom::odom_update_mutex.unlock();
        }

        //get actual sensor readings
        sensors::update_range_sensors();
        sensors::range_readings actual = sensors::distance_readings;

        //create a particle at the current odometry position
        mcl::Particle test_particle;
        test_particle.x = current_pos.x;
        test_particle.y = current_pos.y;
        test_particle.theta = current_pos.theta;
        test_particle.w = 1.0;
        test_particle.W = 1.0;
        test_particle.log_w = 0.0;

        //raycast from this position
        sensors::range_readings expected = raycast(test_particle);

        //print comparison
        printf("\nCurrent Odometry Position:\n");
        printf("  X:     %7.2f\"\n", current_pos.x);
        printf("  Y:     %7.2f\"\n", current_pos.y);
        printf("  Theta: %7.2f° (%7.4f rad)\n", current_pos.theta * (180.0 / M_PI), current_pos.theta);

        printf("\nSensor Readings vs Raycast Predictions:\n");
        printf("  Sensor    | Actual  | Expected | Error   | Status\n");
        printf("  ----------|---------|----------|---------|--------\n");

        //lambda to check each sensor
        auto check_sensor = [](const char* name, double actual, double expected) {
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

        check_sensor("Front", actual.front, expected.front);
        check_sensor("Left", actual.left, expected.left);
        check_sensor("Back", actual.back, expected.back);
        check_sensor("Right", actual.right, expected.right);

        //calculate overall error
        double total_error = 0.0;
        int valid_count = 0;

        if (!std::isnan(actual.front) && !std::isnan(expected.front)) {
            total_error += fabs(actual.front - expected.front);
            valid_count++;
        }
        if (!std::isnan(actual.left) && !std::isnan(expected.left)) {
            total_error += fabs(actual.left - expected.left);
            valid_count++;
        }
        if (!std::isnan(actual.back) && !std::isnan(expected.back)) {
            total_error += fabs(actual.back - expected.back);
            valid_count++;
        }
        if (!std::isnan(actual.right) && !std::isnan(expected.right)) {
            total_error += fabs(actual.right - expected.right);
            valid_count++;
        }

        //print average error and status
        if (valid_count > 0) {
            double avg_error = total_error / valid_count;
            printf("\nAverage Error: %.2f\" (%d/%d sensors valid)\n", avg_error, valid_count, 4);

            if (avg_error < 2.0) {
                printf("Status: ✓ EXCELLENT - Odometry matches sensors well\n");
            } else if (avg_error < 5.0) {
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
    void print_sensor_calibration(double known_distance) {
        //update sensor readings first to get fresh data
        sensors::update_range_sensors();

        sensors::range_readings readings = sensors::distance_readings;

        printf("Sensor Calibration (known distance: %.1f\"):\n", known_distance);
        printf("Raw sensor values (mm):\n");
        printf("  Front raw: %ld mm\n", sensors::distance_sensors.front.get_distance());
        printf("  Left raw:  %ld mm\n", sensors::distance_sensors.left.get_distance());
        printf("  Back raw:  %ld mm\n", sensors::distance_sensors.back.get_distance());
        printf("  Right raw: %ld mm\n", sensors::distance_sensors.right.get_distance());
        printf("\n");

        //calculate and print calibration scale for each sensor
        if (!std::isnan(readings.front) && readings.front > 0.1) {
            printf("  Front: %.2f\" -> scale = %.4f\n",
                   readings.front, known_distance / readings.front);
        } else {
            printf("  Front: sensor error or not connected\n");
        }
        if (!std::isnan(readings.left) && readings.left > 0.1) {
            printf("  Left:  %.2f\" -> scale = %.4f\n",
                   readings.left, known_distance / readings.left);
        } else {
            printf("  Left: sensor error or not connected\n");
        }
        if (!std::isnan(readings.back) && readings.back > 0.1) {
            printf("  Back:  %.2f\" -> scale = %.4f\n",
                   readings.back, known_distance / readings.back);
        } else {
            printf("  Back: sensor error or not connected\n");
        }
        if (!std::isnan(readings.right) && readings.right > 0.1) {
            printf("  Right: %.2f\" -> scale = %.4f\n",
                   readings.right, known_distance / readings.right);
        } else {
            printf("  Right: sensor error or not connected\n");
        }
    }

    //measures and prints range sensor noise statistics
    void measure_range_noise(int num_samples) {
        printf("Measuring range sensor noise (%d samples)...\n", num_samples);
        printf("Keep robot stationary!\n\n");

        //accumulators for statistics
        double sum[4] = {0, 0, 0, 0};
        double sum_sq[4] = {0, 0, 0, 0};
        int count[4] = {0, 0, 0, 0};

        //collect samples
        for (int i = 0; i < num_samples; i++) {
            sensors::update_range_sensors();
            sensors::range_readings readings = sensors::distance_readings;

            if (!std::isnan(readings.front)) {
                sum[0] += readings.front;
                sum_sq[0] += readings.front * readings.front;
                count[0]++;
            }
            if (!std::isnan(readings.left)) {
                sum[1] += readings.left;
                sum_sq[1] += readings.left * readings.left;
                count[1]++;
            }
            if (!std::isnan(readings.back)) {
                sum[2] += readings.back;
                sum_sq[2] += readings.back * readings.back;
                count[2]++;
            }
            if (!std::isnan(readings.right)) {
                sum[3] += readings.right;
                sum_sq[3] += readings.right * readings.right;
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
                double variance = (sum_sq[i] / count[i]) - (mean * mean);
                double stddev = sqrt(variance);

                printf("  %s: mean=%.2f\", stddev=%.3f\" (use %.3f for sigma)\n",
                       names[i], mean, stddev, stddev);
            } else {
                printf("  %s: no valid readings\n", names[i]);
            }
        }
    }
}

