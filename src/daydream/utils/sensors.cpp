//Inclusions
#include "main.h"
#include "control/sensors.hpp"

namespace sensors {

    namespace {
        

        

        double motor_position_to_wheel_inches(double raw_position, pros::MotorUnits units) {
            if (std::isnan(raw_position)) return NAN;

            switch (units) {
                case pros::MotorUnits::degrees:
                    return raw_position * DRIVE_IN_PER_DEG * DRIVE_GEAR_RATIO;
                case pros::MotorUnits::rotations:
                    return raw_position * DRIVE_WHEEL_CIRCUMFERENCE_IN * DRIVE_GEAR_RATIO;
                case pros::MotorUnits::counts:
                default:
                    return NAN;
            }
        }

        double drive_motor_inches(const std::optional<pros::Motor>& motor) {
            if (!motor) return NAN;
            const double raw_position = sanitize_numeric_reading(motor->get_position());
            return motor_position_to_wheel_inches(raw_position, motor->get_encoder_units());
        }

        double tracking_wheel_inches(const std::optional<pros::Rotation>& sensor, double scale) {
            if (!sensor) return NAN;
            const double raw_centideg = sanitize_numeric_reading(sensor->get_position());
            return std::isnan(raw_centideg) ? NAN : raw_centideg * scale;
        }
    }

    range_sensors distance_sensors = {
        pros::Distance(3),  // front
        pros::Distance(2),  // left
        pros::Distance(8),  // back
        pros::Distance(9)}; // right

    odom_sensors chassis_odom_sensors = {
        odom_mode::DRIVE_ENCODERS,
        std::nullopt,
        std::nullopt,
        pros::Motor(-11, pros::MotorGears::blue, pros::MotorUnits::degrees),
        pros::Motor(18, pros::MotorGears::blue, pros::MotorUnits::degrees),
        pros::Imu(21),
        std::nullopt
    };   

    range_readings distance_readings;
    odom_readings odom_values;
    range_calibration distance_calibration = {0.9896,0.9896,0.9896,0.9816};
    // Odometry scale factors:
    // - NEW_2 omniwheel: 2.0" diameter, circumference = 6.283185"
    // - Rotation sensor: 36000 centidegrees/revolution
    // - TPI = 36000 / 6.283185 = 5729.578 centidegrees/inch
    // - IMU scale: degrees to radians = π/180 = 0.017453293
    odom_calibration odom_scale = {
        1.0 / 5729.578,  // parallel tracking TPI (inches per centidegree)
        1.0 / 5729.578,  // perpendicular tracking TPI (inches per centidegree)
        0.017453293,     // IMU scale (radians per degree)
        1.0              // IMU two scale (unused)
    };

    // FIXED: Initialize tracking_offset with actual values
    // These should match your robot's tracking wheel positions
    odom_offset tracking_offset = {0.5, 0.75};  // parallel=0.5", perpendicular=0.75"

    void update_range_sensors () {
        distance_readings.front = sanitize_distance_reading(distance_sensors.front.get_distance()) * MM_TO_IN * distance_calibration.front;
        distance_readings.left = sanitize_distance_reading(distance_sensors.left.get_distance()) * MM_TO_IN * distance_calibration.left;
        distance_readings.back = sanitize_distance_reading(distance_sensors.back.get_distance()) * MM_TO_IN * distance_calibration.back;
        distance_readings.right = sanitize_distance_reading(distance_sensors.right.get_distance()) * MM_TO_IN * distance_calibration.right;
    }

    void update_odom_sensors() {
        double parallel = NAN;
        double perpendicular = NAN;

        // Case 1: parallel tracking wheel exists
        if (chassis_odom_sensors.parallel_tracking) {

            parallel =
                sanitize_numeric_reading(chassis_odom_sensors.parallel_tracking->get_position())
                * odom_scale.parallel_tracking_tpi;
        }

        // Case 2: use drivetrain encoders
        else if (chassis_odom_sensors.drive_left &&
                chassis_odom_sensors.drive_right) {

            const double left = drive_motor_inches(chassis_odom_sensors.drive_left);

            const double right = drive_motor_inches(chassis_odom_sensors.drive_right);

            // forward displacement from differential drive
            parallel = (left + right) * 0.5;
        }

        // perpendicular tracking wheel
        if (chassis_odom_sensors.perpendicular_tracking) {

            perpendicular =
                sanitize_numeric_reading(chassis_odom_sensors.perpendicular_tracking->get_position())
                * odom_scale.perpendicular_tracking_tpi;
        }
        else {

            // no strafe measurement
            perpendicular = 0.0;
        }

        const double heading =
            sanitize_numeric_reading(chassis_odom_sensors.imu_one.get_rotation()) *
            IMU_RAD_PER_DEG;

        odom_values = { parallel, perpendicular, heading };
    }

    void print_range_sensor_debug() {
        const double front_mm = sanitize_distance_reading(distance_sensors.front.get_distance());
        const double left_mm = sanitize_distance_reading(distance_sensors.left.get_distance());
        const double back_mm = sanitize_distance_reading(distance_sensors.back.get_distance());
        const double right_mm = sanitize_distance_reading(distance_sensors.right.get_distance());

        update_range_sensors();

        printf("=== RANGE SENSOR DEBUG ===\n");
        printf("Units: raw=mm, converted=inches, MM_TO_IN=%.6f\n", MM_TO_IN);
        printf("Front: raw=%.2f mm, cal=%.4f, out=%.4f in\n", front_mm, distance_calibration.front, distance_readings.front);
        printf("Left:  raw=%.2f mm, cal=%.4f, out=%.4f in\n", left_mm, distance_calibration.left, distance_readings.left);
        printf("Back:  raw=%.2f mm, cal=%.4f, out=%.4f in\n", back_mm, distance_calibration.back, distance_readings.back);
        printf("Right: raw=%.2f mm, cal=%.4f, out=%.4f in\n", right_mm, distance_calibration.right, distance_readings.right);
        printf("==========================\n");
    }

    void print_odom_sensor_debug() {
        const double left_raw = chassis_odom_sensors.drive_left
            ? sanitize_numeric_reading(chassis_odom_sensors.drive_left->get_position()) : NAN;
        const double right_raw = chassis_odom_sensors.drive_right
            ? sanitize_numeric_reading(chassis_odom_sensors.drive_right->get_position()) : NAN;
        const pros::MotorUnits left_units = chassis_odom_sensors.drive_left
            ? chassis_odom_sensors.drive_left->get_encoder_units() : pros::MotorUnits::invalid;
        const pros::MotorUnits right_units = chassis_odom_sensors.drive_right
            ? chassis_odom_sensors.drive_right->get_encoder_units() : pros::MotorUnits::invalid;
        const double left_inches = drive_motor_inches(chassis_odom_sensors.drive_left);
        const double right_inches = drive_motor_inches(chassis_odom_sensors.drive_right);

        const double parallel_raw_centideg = chassis_odom_sensors.parallel_tracking
            ? sanitize_numeric_reading(chassis_odom_sensors.parallel_tracking->get_position()) : NAN;
        const double perp_raw_centideg = chassis_odom_sensors.perpendicular_tracking
            ? sanitize_numeric_reading(chassis_odom_sensors.perpendicular_tracking->get_position()) : NAN;
        const double parallel_inches = tracking_wheel_inches(chassis_odom_sensors.parallel_tracking, odom_scale.parallel_tracking_tpi);
        const double perp_inches = tracking_wheel_inches(chassis_odom_sensors.perpendicular_tracking, odom_scale.perpendicular_tracking_tpi);

        const double imu_heading_deg = sanitize_numeric_reading(chassis_odom_sensors.imu_one.get_heading());
        const double imu_rotation_deg = sanitize_numeric_reading(chassis_odom_sensors.imu_one.get_rotation());

        update_odom_sensors();

        printf("=== ODOM SENSOR DEBUG ===\n");
        printf("Mode: %s\n", odom_mode_name(chassis_odom_sensors.mode));
        printf("Drive encoder conversion: wheel_diam=%.3f in, wheel_circ=%.6f in, ext_ratio=%.6f\n",
               DRIVE_WHEEL_DIAMETER_IN, DRIVE_WHEEL_CIRCUMFERENCE_IN, DRIVE_GEAR_RATIO);
        printf("Left drive:  raw=%.4f %s, conv=%.4f in\n", left_raw, motor_units_name(left_units), left_inches);
        printf("Right drive: raw=%.4f %s, conv=%.4f in\n", right_raw, motor_units_name(right_units), right_inches);
        printf("Drive avg parallel=%.4f in, left-right diff=%.4f in\n",
               (left_inches + right_inches) * 0.5, left_inches - right_inches);
        printf("Parallel tracker: raw=%.4f cdeg, conv=%.4f in\n", parallel_raw_centideg, parallel_inches);
        printf("Perp tracker:     raw=%.4f cdeg, conv=%.4f in\n", perp_raw_centideg, perp_inches);
        printf("IMU heading=%.4f deg (wrapped), rotation=%.4f deg, rotation=%.6f rad\n",
               imu_heading_deg, imu_rotation_deg, imu_rotation_deg * IMU_RAD_PER_DEG);
        printf("Computed odom_values: parallel=%.4f in, perp=%.4f in, heading=%.6f rad\n",
               odom_values.parallel_tracking, odom_values.perpendicular_tracking, odom_values.heading);
        printf("=========================\n");
    }
}

