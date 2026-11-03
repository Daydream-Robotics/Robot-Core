#pragma once

// Robot device assignments shared by both ends of the link.
constexpr int PARALLEL_TRACKING_WHEEL_PORT = 6;
constexpr int PERPENDICULAR_TRACKING_WHEEL_PORT = 10;
#define LEFT_DRIVE_WHEEL_PORTS {-14, -12, -13}
#define RIGHT_DRIVE_WHEEL_PORTS {17, 18, 19}
constexpr char BALL_BLOCKER_PORT = 'A';
constexpr char SCORING_LIFTER = 'B';
constexpr char MATCHLOADER_PORT = 'C';
constexpr char DESCORE_PORT = 'D';
