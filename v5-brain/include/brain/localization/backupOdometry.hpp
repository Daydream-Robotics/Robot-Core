/** @file backupOdometry.hpp
 *  @brief Brain-side adapter that updates shared odometry from sensor readings.
 */

#pragma once

#include "brain/types.hpp"
#include "odometry.hpp"

namespace brain {

/** @brief Provides a selectable local odometry estimate from tracking data.
 *  @note Call methods from one task, or synchronize access externally. When used
 *  by AcquisitionTask, read the published pose from the BrainState mailbox.
 */
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
    /** @brief Check whether the estimate has remained continuous.
     *  @return True after a sample is integrated without a subsequent gap.
     */
    bool valid() const;
    /** @brief Apply sensor deltas to shared odometry when enabled.
     *  @param sensors Snapshot containing the latest wheel and heading deltas.
     *  @details Expects SensorService snapshots with the parallel wheel in slot 0
     *  and perpendicular wheel in slot 1. Wheel travel is in configured length
     *  units. Call once per acquisition cycle. Motion lost during a sensor outage
     *  cannot be recovered; reset the pose to restore validity.
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
    shared::Odometry& m_odometry;
    /// Whether @ref update applies incoming sensor snapshots.
    bool m_enabled = false;
    /// Whether the pose has remained continuous since the last reset.
    bool m_valid = false;
    /// Whether a usable sample has been integrated since the last reset.
    bool m_hasReadySample = false;
};

} // namespace brain
