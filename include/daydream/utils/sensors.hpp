#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

//Inclusions
#include "api.h"
#include <optional>

//Created namespace sensors
namespace sensors {

    

    //odometry sensor objects (tracking wheels and IMU)
    


    

    //distance sensor objects
    extern range_sensors distance_sensors;
    //odometry sensor objects
    extern odom_sensors chassis_odom_sensors;
    //current odometry readings
    extern odom_readings odom_values;
    //tracking wheel offsets from robot center
    extern odom_offset tracking_offset;
    //current distance sensor readings
    extern range_readings distance_readings;

    //updates odometry sensor readings
    void update_odom_sensors();
    //updates distance sensor readings
    void update_range_sensors();
    //prints raw + converted range sensor values for unit checking
    void print_range_sensor_debug();
    //prints raw + converted odom sensor values for sign/unit checking
    void print_odom_sensor_debug();
}
#endif

