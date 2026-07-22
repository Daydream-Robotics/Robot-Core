## Steps
1. Run MPCSerial::identifyMotorModel() in opcontrol to get the a and b values that you set in microcontroller/src/microcontroller_main.cpp
2. set the other parameters in microcontroller/src/microcontroller_main.cpp, specifically r, L, omega_motor_max
3. If wanted change p_x, p_y, p_theta, q_u, q_zero_mult, q_final_mult, p_final_mult, delta_u_max, delta_u_min (tuning steps below)
4. read over microcontroller/README.md to set up microcontroller
5. copy over microcontroller/ to the microcontroller
6. run ./MPCController on the microcontroller after building (look at microcontroller/README.md)
7. run this program on the brain


## Tuning:
### delta_u_max and delta_u_min (Slew Rate Limits) (V/step):
- If too high: The motors will receive instant $12\text{V}$ spikes on acceleration, causing the wheels to spin out, slip on the tiles, and draw excessive battery current.
- If too low: The robot will feel sluggish, taking too long to speed up or slow down.
- Tuning Policy: Start at $\pm 4.0\text{V}$. If you notice wheel slippage on startup or during hard turns, decrease this value to 3 or 2.5.

### q_u (Control Input Voltage Penalty)
This is the penalty weight of change in control input.
- If too low: The robot handles path variations aggressively, but the wheels may oscillate violently because it costs the optimizer virtually nothing to swing from $+12\text{ V}$ to $-12\text{ V}$.
- If too high: The robot drives very smoothly but turns wide on corners and fails to correct small errors because it values maintaining constant voltage over tracking precision.
- Tuning Policy: Start at 0.01 and vary based on conditions above

### p_y
- Start with pY = 15.0: If during straight runs the robot drifts laterally or ignores path displacement, increase pY by 5.

### p_theta
- Start with p_theta = 100.0 If the robot tracks straight lines perfectly but cuts corners on sharp turns, increase by 50



How to do main properly:
at top:
```
MPCSerial* mpc_serial_controller = nullptr;
PathFollower* pathFollower = nullptr;
```
in ```initialize()```:
```
SerialProtocol::setUpVexSerial();
    pros::lcd::initialize();
    pros::lcd::print(0, "MPC Serial Initialization");

    Logger::getInstance().init("/usd/log.txt");
	LOG("Program Start");
    
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    printf("[MAIN] Loading paths...\n");
	paths = Path::buildAllPathsFromJerryIO("/usd/path.jerryio.txt");
	printf("[MAIN] Paths loaded: %zu\n", paths.size());
	if (paths.size() != PathName::COUNT) {
		printf("[MAIN] ERROR: Path count mismatch\n");
		pros::lcd::print(1, "ERROR: Path count mismatch");
		pros::lcd::print(2, "Expected: %d, Actual: %d", PathName::COUNT, paths.size());
	} else {
		printf("[MAIN] Trajectories Loaded successfully\n");
		pros::lcd::print(1, "Trajectories Loaded");
	}


    constexpr double GEAR_RATIO = 450.0 / 600.0;
    MPCSerial::Params mpc_serial_params(0.02, GEAR_RATIO);
    mpc_serial_controller = new MPCSerial(mpc_serial_params);
    pathFollower = new PathFollower(*mpc_serial_controller);

    // object handler setup
	// pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");
	printf("[MAIN] Initialize complete\n");
```

in ```auton()``:
```
 printf("[MAIN] Starting autonomous()\n");
	
	if (paths.size() <= PathName::FIRST_PATH) {
		printf("[MAIN-ERROR] FIRST_PATH index out of bounds! Array size is %zu\n", paths.size());
		return;
	}

	printf("[MAIN] Setting FIRST_PATH...\n");
	pathFollower->setPath(paths[PathName::FIRST_PATH]);
	printf("[MAIN] FIRST_PATH set. Tracking...\n");
	while (not pathFollower->step()) {
		pros::delay(20);
	}
	printf("[MAIN] FIRST_PATH tracking complete.\n");

	printf("[MAIN] Delaying 2000ms...\n");
	pros::delay(2000);

	if (paths.size() <= PathName::SECOND_PATH) {
		printf("[MAIN-ERROR] SECOND_PATH index out of bounds! Array size is %zu\n", paths.size());
		return;
	}

	printf("[MAIN] Setting SECOND_PATH...\n");
	pathFollower->setPath(paths[PathName::SECOND_PATH]);
	printf("[MAIN] SECOND_PATH set. Tracking...\n");
	while (not pathFollower->step()) {
		pros::delay(20);
	}
	printf("[MAIN] SECOND_PATH tracking complete.\n");
```