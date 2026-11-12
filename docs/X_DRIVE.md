# Future X-Drive Driver Code





```cpp
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		// Right stick: direction of travel, left stick X: rotation
		int forward = master.get_analog(ANALOG_LEFT_Y);
		int strafe = master.get_analog(ANALOG_LEFT_X);
		int turn = master.get_analog(ANALOG_RIGHT_X);
        // Small deadzone to stop drift from the sticks
		if (abs(forward) < 5) forward = 0;
		if (abs(strafe)  < 5) strafe  = 0;
		if (abs(turn)    < 5) turn    = 0;
		

		// X-drive mixing
		double fl = forward + strafe + turn;
		double bl = forward - strafe + turn;
		double fr = forward - strafe - turn;
		double br = forward + strafe - turn;



		// Scale down if any value exceeds 127 so direction is preserved
		double maxVal = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 127.0});
		double scale = 127.0 / maxVal;

		frontLeft.move(fl * scale);
		frontRight.move(fr * scale);
		backLeft.move(bl * scale);
		backRight.move(br * scale);

		pros::delay(20);
	}
}
```


