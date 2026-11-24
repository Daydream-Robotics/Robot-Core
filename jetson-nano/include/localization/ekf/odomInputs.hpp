#pragma once

#include <cstdint>
#include <optional>

#include "localization/imu/bno085.hpp"
#include "protocol/serial.hpp"

/**
 * @brief One cycle of odometry increments, in robot units.
 */
struct OdomInputs {
    int64_t tUs = 0; ///< end of this cycle, Nano steady_clock (us)
    double dt = 0.0; ///< seconds since the previous OdomInputs

    double dsPar = 0.0; ///< parallel tracking wheel travel (in)
    double dsPerp = 0.0; ///< perpendicular tracking wheel travel, positive along +X at heading 0 (in)
    double dsLeft = 0.0; ///< left drive travel from IMEs (in)
    double dsRight = 0.0; ///< right drive travel from IMEs (in)
    double dTheta = 0.0; ///< IMU heading change, CW+ (rad)

    bool parValid = false; ///< dsPar can be used this cycle
    bool perpValid = false; ///< dsPerp can be used this cycle
    bool imeValid = false; ///< dsLeft and dsRight can be used this cycle
    bool imuValid = false; ///< dTheta can be used this cycle
};

/**
 * @brief Converts link samples into OdomInputs.
 */
class OdomInputAdapter {
public:
    /**
     * @brief Builds the increments since the previous sample.
     *
     * @param sample Latest sample from the brain.
     * @param imu BNO085 driver, for the heading change over the same interval.
     * @return std::optional<OdomInputs> Nothing for the first sample (no previous one to difference).
     */
    std::optional<OdomInputs> convert(const SerialLink::Sample& sample, const BNO085& imu);

    /**
     * @brief Forgets the previous sample (call on link reconnect or brain restart).
     */
    void reset();

private:
    std::optional<int64_t> m_lastTUs; ///< tNanoUs of the previous sample, empty until the first one
};
