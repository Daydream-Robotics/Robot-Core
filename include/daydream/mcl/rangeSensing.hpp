#ifndef _RANGE_SENSING_HPP_
#define _RANGE_SENSING_HPP_

//Inclusions
#include "api.h"

//forward declarations to avoid circular dependency
namespace mcl {
    struct Particle;
}

struct RangeReadings;

//Created class RangeSensing
class RangeSensing {
public:
    //line segment structure for walls and obstacles
    struct LineSegment {
        double x1;
        double x2;
        double y1;
        double y2;
    };

    //precomputed segment properties for fast raycasting
    struct PrecomputedSegment {
        float x1, y1;           //start point of line segment
        float dx, dy;           //direction vector (x2-x1, y2-y1)
    };

    //2D position structure with validity flag
    struct Position2d {
        double x;
        double y;
        double theta;
        bool valid;
    };

    //sin/cos lookup table size (0.1 degree resolution)
    static constexpr int ANGLE_LUT_SIZE = 3600;

    //number of line segments for raycasting (walls + matchloaders)
    static constexpr int NUM_SEGMENTS = 16;

    //sensor direction offsets relative to robot heading
    //coordinate system: 0°=left, 90°=up, 180°=right, 270°=down
    //front sensor: 0° offset from robot heading
    //left sensor: +90° offset (CCW) from robot heading
    //back sensor: +180° offset from robot heading
    //right sensor: +270° offset (or -90°) from robot heading
    static constexpr int SIN_HEADINGS[4] = {1, 0, -1, 0};   // front(0°), left(90°), back(180°), right(270°)
    static constexpr int COS_HEADINGS[4] = {0, 1, 0, -1};   // front(0°), left(90°), back(180°), right(270°)

    //performs raycasting from particle position to find expected sensor readings
    RangeReadings raycast(const mcl::Particle& particle);
    //optimized raycast for when all particles have same theta
    RangeReadings raycastFixedTheta(const mcl::Particle& particle,
                                    float headingSin, float headingCos,
                                    const float cosSensors[4], const float sinSensors[4]);

    //initializes lookup tables and precomputed data
    void initRangeSystem();
    //fast sin/cos lookup using precomputed table
    void fastSincos(float theta, float& sinOut, float& cosOut);
    //prints estimated position from sensor readings
    void printPositionFromSensors();
    //prints sensor calibration factors for known distance
    void printSensorCalibration(double knownDistance);
    //measures and prints range sensor noise statistics
    void measureRangeNoise(int numSamples);

    //checks raycast accuracy at current position
    void checkRaycastAccuracy();

    //calculates robot position using 1 or 2 sensors + IMU heading
    //beams from sensors to walls and calculates robot center position
    //sensor1Idx: sensor index (0=front, 1=left, 2=back, 3=right) or -1 to skip
    //sensor2Idx: sensor index (0=front, 1=left, 2=back, 3=right) or -1 to skip
    //silent: if true, disables debug printout (default: false)
    //returns robot position (x, y, theta). x or y will be NAN if not calculated
    Position2d calculatePositionFrom2Sensors(int sensor1Idx, int sensor2Idx, bool silent = false);

private:
    //converts chassis/field heading (90 deg = +x) to math heading
    static float fieldHeadingToMathHeading(float fieldHeadingRad);

    //sensor physical offsets from robot center in inches
    static constexpr float SENSOR_OFFSET_X[4] = {-2.8, -5, -4.0, -5};      //front, left, back, right
    static constexpr float SENSOR_OFFSET_Y[4] = {5.0, 4.75, -5.0, -4.75};  //front, left, back, right

    //field boundaries in inches
    static constexpr double X_MIN = -70.0;
    static constexpr double X_MAX = 70.0;
    static constexpr double Y_MIN = -70.0;
    static constexpr double Y_MAX = 70.0;

    //matchloader dimensions in inches
    static constexpr double MATCHLOADER_DIAMETER = 4.17;
    static constexpr double MATCHLOADER_RADIUS = MATCHLOADER_DIAMETER/2;

    //matchloader center x positions in inches
    static constexpr double ML_X_LEFT  = -46.6;
    static constexpr double ML_X_RIGHT =  46.6;

    //matchloader edge x positions
    static constexpr double XL_L = ML_X_LEFT  - MATCHLOADER_RADIUS;  // -48.685
    static constexpr double XR_L = ML_X_LEFT  + MATCHLOADER_RADIUS;  // -44.515
    static constexpr double XL_R = ML_X_RIGHT - MATCHLOADER_RADIUS;  //  44.515
    static constexpr double XR_R = ML_X_RIGHT + MATCHLOADER_RADIUS;  //  48.685

    //matchloader edge y positions
    static constexpr double Y_TOP_BOTTOM =  70.0 - MATCHLOADER_DIAMETER;   //  65.83
    static constexpr double Y_BOT_TOP    = -70.0 + MATCHLOADER_DIAMETER;   // -65.83

    //field boundary walls (x1, x2, y1, y2) - field is 140" × 140"
    static constexpr LineSegment WALL_LEFT   = {-70, -70, -70, 70};   // left wall (x = -70)
    static constexpr LineSegment WALL_RIGHT  = {70, 70, -70, 70};     // right wall (x = +70)
    static constexpr LineSegment WALL_TOP    = {-70, 70, 70, 70};     // top wall (y = +70)
    static constexpr LineSegment WALL_BOTTOM = {-70, 70, -70, -70};   // bottom wall (y = -70)

    //matchloader obstacle segments - top left
    static constexpr LineSegment MATCHLOADER_TOP_LEFT_LEFT   = {XL_L, XL_L, Y_TOP_BOTTOM, 70};
    static constexpr LineSegment MATCHLOADER_TOP_LEFT_RIGHT  = {XR_L, XR_L, Y_TOP_BOTTOM, 70};
    static constexpr LineSegment MATCHLOADER_TOP_LEFT_BOTTOM = {XL_L, XR_L, Y_TOP_BOTTOM, Y_TOP_BOTTOM};

    //matchloader obstacle segments - top right
    static constexpr LineSegment MATCHLOADER_TOP_RIGHT_LEFT   = {XL_R, XL_R, Y_TOP_BOTTOM, 70};
    static constexpr LineSegment MATCHLOADER_TOP_RIGHT_RIGHT  = {XR_R, XR_R, Y_TOP_BOTTOM, 70};
    static constexpr LineSegment MATCHLOADER_TOP_RIGHT_BOTTOM = {XL_R, XR_R, Y_TOP_BOTTOM, Y_TOP_BOTTOM};

    //matchloader obstacle segments - bottom left
    static constexpr LineSegment MATCHLOADER_BOTTOM_LEFT_LEFT   = {XL_L, XL_L, -70, Y_BOT_TOP};
    static constexpr LineSegment MATCHLOADER_BOTTOM_LEFT_RIGHT  = {XR_L, XR_L, -70, Y_BOT_TOP};
    static constexpr LineSegment MATCHLOADER_BOTTOM_LEFT_TOP    = {XL_L, XR_L, Y_BOT_TOP, Y_BOT_TOP};

    //matchloader obstacle segments - bottom right
    static constexpr LineSegment MATCHLOADER_BOTTOM_RIGHT_LEFT   = {XL_R, XL_R, -70, Y_BOT_TOP};
    static constexpr LineSegment MATCHLOADER_BOTTOM_RIGHT_RIGHT  = {XR_R, XR_R, -70, Y_BOT_TOP};
    static constexpr LineSegment MATCHLOADER_BOTTOM_RIGHT_TOP    = {XL_R, XR_R, Y_BOT_TOP, Y_BOT_TOP};

    //all line segments to test for raycasting
    static constexpr LineSegment LINE_SEGMENTS[] = {
        WALL_LEFT,
        WALL_RIGHT,
        WALL_TOP,
        WALL_BOTTOM,

        MATCHLOADER_TOP_LEFT_LEFT,
        MATCHLOADER_TOP_LEFT_RIGHT,
        MATCHLOADER_TOP_LEFT_BOTTOM,

        MATCHLOADER_TOP_RIGHT_LEFT,
        MATCHLOADER_TOP_RIGHT_RIGHT,
        MATCHLOADER_TOP_RIGHT_BOTTOM,

        MATCHLOADER_BOTTOM_LEFT_LEFT,
        MATCHLOADER_BOTTOM_LEFT_RIGHT,
        MATCHLOADER_BOTTOM_LEFT_TOP,

        MATCHLOADER_BOTTOM_RIGHT_LEFT,
        MATCHLOADER_BOTTOM_RIGHT_RIGHT,
        MATCHLOADER_BOTTOM_RIGHT_TOP
    };

    //number of line segments must match NUM_SEGMENTS
    static_assert(sizeof(LINE_SEGMENTS) / sizeof(LineSegment) == NUM_SEGMENTS,
                  "RangeSensing::NUM_SEGMENTS does not match LINE_SEGMENTS");

    //precomputed segment properties (initialized in initRangeSystem)
    PrecomputedSegment m_precomputedSegments[NUM_SEGMENTS];

    //sin/cos lookup tables for fast trig calculations
    float m_sinLut[ANGLE_LUT_SIZE];
    float m_cosLut[ANGLE_LUT_SIZE];
};
#endif

