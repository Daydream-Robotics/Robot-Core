#ifndef _RANGE_HPP_
#define _RANGE_HPP_

//Inclusions
#include "api.h"

//forward declarations to avoid circular dependency
namespace mcl {
    struct Particle;
}

namespace sensors {
    struct range_readings;
}

//Created namespace range
namespace range {

    //performs raycasting from particle position to find expected sensor readings
    sensors::range_readings raycast(const mcl::Particle& particle);
    //optimized raycast for when all particles have same theta
    sensors::range_readings raycast_fixed_theta(const mcl::Particle& particle,
                                                 float heading_sin, float heading_cos,
                                                 const float cos_sensors[4], const float sin_sensors[4]);

    //line segment structure for walls and obstacles
    struct line_segment {
        double x_1;
        double x_2;
        double y_1;
        double y_2;
    };

    //precomputed segment properties for fast raycasting
    struct precomputed_segment {
        float x1, y1;           //start point of line segment
        float dx, dy;           //direction vector (x2-x1, y2-y1)
    };

    //sin/cos lookup table size (0.1 degree resolution)
    constexpr int ANGLE_LUT_SIZE = 3600;
    //sin/cos lookup tables for fast trig calculations
    extern float sin_lut[ANGLE_LUT_SIZE];
    extern float cos_lut[ANGLE_LUT_SIZE];

    //sensor direction offsets (front=0°, left=90°, back=180°, right=270°)
    extern const int cos_headings[4];
    extern const int sin_headings[4];

    //initializes lookup tables and precomputed data
    void init_range_system();
    //fast sin/cos lookup using precomputed table
    void fast_sincos(float theta, float& sin_out, float& cos_out);
    //prints estimated position from sensor readings
    void print_position_from_sensors();
    //prints sensor calibration factors for known distance
    void print_sensor_calibration(double known_distance);
    //measures and prints range sensor noise statistics
    void measure_range_noise(int num_samples);

    //checks raycast accuracy at current position
    void check_raycast_accuracy();

    //2D position structure with validity flag
    struct position_2d {
        double x;
        double y;
        double theta;
        bool valid;
    };

    //calculates robot position using 1 or 2 sensors + IMU heading
    //beams from sensors to walls and calculates robot center position
    //sensor1_idx: sensor index (0=front, 1=left, 2=back, 3=right) or -1 to skip
    //sensor2_idx: sensor index (0=front, 1=left, 2=back, 3=right) or -1 to skip
    //silent: if true, disables debug printout (default: false)
    //returns robot position (x, y, theta). x or y will be NAN if not calculated
    position_2d calculate_position_from_2_sensors(int sensor1_idx, int sensor2_idx, bool silent = false);
}
#endif

