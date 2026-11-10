/** @file nanoLink.hpp
 *  @brief Brain hardware and link boundary for the shared serial protocol.
 */
#pragma once

#include "brain/configuration.hpp"
#include "brain/types.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <vector>

namespace brain {

/** Uses existing devices owned elsewhere; all supplied devices must outlive this object.
 * The owner must coordinate access from other tasks with serial activity.
 */
class NanoLink {
public:
    NanoLink(pros::Controller& controller,
             const std::vector<pros::MotorGroup*>& groups,
             const std::vector<pros::Rotation*>& wheels,
             const std::array<pros::adi::DigitalOut*, packets::MAX_ADI>& outputs);
    ~NanoLink();
    NanoLink(const NanoLink&) = delete;
    NanoLink& operator=(const NanoLink&) = delete;
    packets::Error applyConfiguration(const BrainConfiguration& config);
    BrainState sample();
    packets::Error applyOutputs(const NanoCommand& command);
    packets::Error executeRequest(const packets::CommandFixed& command);
    void brakeGroups();
private:
    void brakeGroupsUnlocked();
    struct Group {
        pros::MotorGroup* motor = nullptr;
        std::vector<double> last;
        std::vector<bool> valid;
        double position = 0;
        pros::MotorGears gear = pros::MotorGears::green;
        pros::MotorBrake brakeMode = pros::MotorBrake::coast;
        bool missing = false;
    };
    struct Wheel {
        pros::Rotation* sensor = nullptr;
        double last = 0, position = 0;
        bool valid = false, reversed = false, missing = false;
    };
    pros::Controller& controller_;
    std::vector<Group> groups_;
    std::vector<Wheel> wheels_;
    std::array<pros::adi::DigitalOut*, packets::MAX_ADI> outputs_{};
    uint16_t enabled_ = 0, activeLow_ = 0, level_ = 0;
    uint8_t expander_ = 0;
    bool expanderMissing_ = false;
    pros::Mutex mutex_;
};

} // namespace brain
