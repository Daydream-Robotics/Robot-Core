#pragma once

#include "brain/types.hpp"
#include "odometry.hpp"

namespace brain {

class BackupOdometry {
public:
    explicit BackupOdometry(shared::Odometry& odometry);

    void setEnabled(bool enabled);
    bool enabled() const;
    void update(const SensorSnapshot& sensors);
    shared::Pose pose() const;
    void reset(const shared::Pose& pose);

private:
    shared::Odometry& odometry_;
    bool enabled_ = false;
};

} // namespace brain
