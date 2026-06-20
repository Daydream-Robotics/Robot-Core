1.) Run MPCSerial::identifyMotorModel() in opcontrol to get the a and b values that you set in microcontroller/src/microcontroller_main.cpp
2.) set the other parameters in microcontroller/src/microcontroller_main.cpp, specifically r, L, omega_motor_max
3.) If wanted change p_x, p_y, p_theta, q_u, q_zero_mult, q_final_mult, p_final_mult, delta_u_max, delta_u_min (tuning steps below)
4.) read over microcontroller/README.md to set up microcontroller
5.) copy over microcontroller/ to the microcontroller
6.) run ./MPCController on the microcontroller after building (look at microcontroller/README.md)
7.)run this program on the brain