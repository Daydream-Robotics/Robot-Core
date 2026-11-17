#include "perception/lidar/lidarExtrinsics.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double epsilon = 1e-9) {
    return std::fabs(a - b) < epsilon;
}

int main(int argc, char** argv) {
    assert(argc == 2); // Pass the path to lidar_mounts.ini.
    std::vector<LidarMount> loaded;
    assert(loadLidarMounts(argv[1], loaded));
    assert(loaded.size() == 2);
    assert(loaded[0].sensor_id == 0 && loaded[1].sensor_id == 1);
    assert(near(loaded[0].footprint.margin_in, 1.0));

    LidarMount mount;
    mount.sensor_id = 7;
    mount.x_in = 10.0;
    mount.y_in = -5.0;
    mount.psi_rad = kPi / 2.0;
    mount.r_min_in = 1.0;
    mount.r_max_in = 100.0;
    LidarTransform transformer(mount);
    Scan scan;
    scan.sensor_id = 7;
    scan.t_start_us = 100;
    scan.t_end_us = 200;
    scan.points.push_back({20.0, 0.0, 125, 10});
    scan.points.push_back({0.0, 0.0, 126, 10}); // invalid zero distance
    scan.points.push_back({101.0, 0.0, 127, 10}); // > max range
    auto result = transformer.toRobotFrame(scan);
    assert(result.sensor_id == 7);
    assert(result.t_start_us == 100 && result.t_end_us == 200);
    assert(result.points.size() == 1);
    assert(near(result.points[0].x_in, 10.0));
    assert(near(result.points[0].y_in, 15.0));
    assert(result.points[0].stamp_us == 125);

    // +pi/2 sensor bearing should be +Y when the sensor is unrotated.
    mount.psi_rad = 0;
    LidarTransform leftTransform(mount);
    scan.points = {{10.0, kPi / 2.0, 150, 9}};
    result = leftTransform.toRobotFrame(scan);
    assert(result.points.size() == 1);
    assert(near(result.points[0].x_in, 10.0));
    assert(near(result.points[0].y_in, 5.0));

    // A blocked bearing interval wrapping across -pi / +pi.
    mount.blocked_bearings = {{170.0 * kPi / 180.0, -170.0 * kPi / 180.0}};
    LidarTransform masked(mount);
    scan.points = {{20.0, 179.0 * kPi / 180.0, 151, 10},
                   {20.0, -179.0 * kPi / 180.0, 152, 10},
                   {20.0, 0.0, 153, 10}};
    result = masked.toRobotFrame(scan);
    assert(result.points.size() == 1 && result.points[0].stamp_us == 153);

    // Chassis footprint with margin should reject internal coordinates.
    mount.blocked_bearings.clear();
    mount.x_in = 0.0;
    mount.y_in = 0.0;
    mount.footprint = {12, 12, 10, 10, 1};
    LidarTransform filtered(mount);
    scan.points = {{10, 0, 151, 9}, {13, 0, 152, 9}, {20, 0, 153, 9}};
    result = filtered.toRobotFrame(scan);
    assert(result.points.size() == 1 && result.points[0].stamp_us == 153);

    // Sensor-mismatch fails loudly; it must not silently misalign the cloud.
    bool mismatchCaught = false;
    try { (void)LidarTransform(mount).toRobotFrame(Scan{}); }
    catch (const std::invalid_argument&) { mismatchCaught = true; }
    assert(mismatchCaught);

    // Two sensor frames looking at the same wall point => identical robot XY.
    LidarMount a;
    a.sensor_id = 0;
    a.x_in = 5;
    a.r_min_in = 1;
    a.r_max_in = 100;
    LidarMount b = a;
    b.sensor_id = 1;
    b.x_in = -5;
    b.psi_rad = kPi;
    Scan sa, sb;
    sa.sensor_id = 0;
    sb.sensor_id = 1;
    sa.points.push_back({15, 0, 100, 0});
    sb.points.push_back({25, kPi, 100, 0});
    const auto pa = LidarTransform(a).toRobotFrame(sa).points.at(0);
    const auto pb = LidarTransform(b).toRobotFrame(sb).points.at(0);
    assert(near(pa.x_in, 20) && near(pb.x_in, 20));
    assert(near(pa.y_in, 0) && near(pb.y_in, 0));

    // Bad config should never replace a previously loaded good set.
    const std::string badConfig = "/tmp/lidar_extrinsics_bad_test.ini";
    { std::ofstream bad(badConfig); bad << "[robot]\nfront_in=5\n"; }
    assert(!loadLidarMounts(badConfig, loaded));
    assert(loaded.size() == 2);

    std::cout << "All LiDAR extrinsics synthetic tests passed.\n";
}
