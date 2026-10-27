/** @file backupOdometry.hpp
 *  @brief Brain-side adapter that updates shared odometry from sensor readings.
 */

#pragma once

#include "brain/types.hpp"
#include "odometry.hpp"

namespace brain {

/** @brief Provides a selectable local odometry estimate from tracking data. */
class BackupOdometry {
public:
    /** @brief Bind the Brain adapter to the shared odometry calculation.
     *  @param odometry Shared odometry instance to update and query.
     *  @note The referenced odometry object must outlive this adapter.
     */
    explicit BackupOdometry(shared::Odometry& odometry);

    /** @brief Enable or disable updates from Brain sensor snapshots.
     *  @param enabled True to update the shared odometry from new samples.
     */
    void setEnabled(bool enabled);
    /** @brief Check whether sensor updates are enabled.
     *  @return True when this adapter accepts updates.
     */
    bool enabled() const;
    /** @brief Apply sensor deltas to shared odometry when enabled.
     *  @param sensors Snapshot containing the latest wheel and heading deltas.
     */
    void update(const SensorSnapshot& sensors);
    /** @brief Get the current shared odometry pose.
     *  @return Current robot pose in the shared configuration's length units.
     */
    shared::Pose pose() const;
    /** @brief Reset the shared odometry estimate.
     *  @param pose New pose to use as the odometry origin/state.
     */
    void reset(const shared::Pose& pose);

private:
    /// Non-owning shared odometry instance used for pose estimation.
    shared::Odometry& odometry_;
    /// Whether @ref update applies incoming sensor snapshots.
    bool enabled_ = false;
};

} // namespace brain
