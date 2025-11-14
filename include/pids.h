
/**
 * Gets the yaw value in quaternions, which is significantly 
 * more accurate than the simple yaw or heading functions.
 */
double get_yaw_quaternions();

void turn_pid(double target, double weightAdjustment = 0);