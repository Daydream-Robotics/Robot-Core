/** @file driverInputReader.cpp
 *  @brief Reads the state of a PROS controller and converts it into a DriverInput struct.
 */

#include "brain/control/driverInputReader.hpp"


#include "pros/error.h"   // for PROS_ERR
#include "pros/misc.h"    
namespace brain {

DriverInputReader::DriverInputReader(pros::Controller& controller)
    : controller_(controller) {}

DriverInput DriverInputReader::read() const {
    DriverInput input{};   // Starts as: axes all 0, buttons 0, connected false

    input.connected = controller_.is_connected() == 1;

    // If not connected, return input early (sticks stay centered)
    if (!input.connected) {
        return input;
    }

    // Loop i = 0..3 over the axes (LX, LY, RX, RY)
    //       - get the PROS axis for i
    //       - read it with controller_.get_analog(...)
    //       - store it in input.axes[i] (PROS_ERR -> 0)
    for(int i = 0; i < 4; ++i) {
        pros::controller_analog_e_t axis;
        switch (i) {
            case 0: axis = pros::E_CONTROLLER_ANALOG_LEFT_X; break;
            case 1: axis = pros::E_CONTROLLER_ANALOG_LEFT_Y; break;
            case 2: axis = pros::E_CONTROLLER_ANALOG_RIGHT_X; break;
            case 3: axis = pros::E_CONTROLLER_ANALOG_RIGHT_Y; break;
        }
        int value = controller_.get_analog(axis);
        if (value == PROS_ERR) {
            value = 0; // default to 0 on error
        }
        input.axes[i] = static_cast<std::int8_t>(value);
    }

    // Loop i = 0..11 over the buttons (L1 L2 R1 R2 Up Down Left Right X B Y A)
    //       - get the PROS button for i
    //       - if controller_.get_digital(...) == 1, set bit i in input.buttons
    for(int i = 0; i < 12; ++i) {
        pros::controller_digital_e_t button;
        switch (i) {
            case 0: button = pros::E_CONTROLLER_DIGITAL_L1; break;
            case 1: button = pros::E_CONTROLLER_DIGITAL_L2; break;
            case 2: button = pros::E_CONTROLLER_DIGITAL_R1; break;
            case 3: button = pros::E_CONTROLLER_DIGITAL_R2; break;
            case 4: button = pros::E_CONTROLLER_DIGITAL_UP; break;
            case 5: button = pros::E_CONTROLLER_DIGITAL_DOWN; break;
            case 6: button = pros::E_CONTROLLER_DIGITAL_LEFT; break;
            case 7: button = pros::E_CONTROLLER_DIGITAL_RIGHT; break;
            case 8: button = pros::E_CONTROLLER_DIGITAL_X; break;
            case 9: button = pros::E_CONTROLLER_DIGITAL_B; break;
            case 10: button = pros::E_CONTROLLER_DIGITAL_Y; break;
            case 11: button = pros::E_CONTROLLER_DIGITAL_A; break;
        }
        if (controller_.get_digital(button) == 1) {
            input.buttons |= (1 << i);
        }
    }

    return input;
}

} // namespace brain