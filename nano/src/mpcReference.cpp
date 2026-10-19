#include "mpcReference.hpp"

template<std::size_t F>
MPCReference<F>::MPCReference(const Params& params)
    : m_params(params) {}

//build reference trajectory (see mpc_ref::rollout)
template<std::size_t F>
typename MPCReference<F>::ZRef MPCReference<F>::build(const Pose& currentPose, const ALS_Path& path, std::size_t idx, bool reverse, double omega_L, double omega_R) {
    const std::vector<Sample>& samples = path.getSamples();
    auto sampleAt = [&path](double s) { return path.sampleAtArcLength(s); };

    float z_raw[(F + 1) * 3];
    mpc_ref::rollout<F>(m_params, sampleAt, samples[idx].s, samples.back().s, currentPose.theta, reverse, omega_L, omega_R, m_v_ref, z_raw);

    ZRef z;
    for (std::size_t i = 0; i < (F + 1) * 3; i++) {
        z(i) = static_cast<double>(z_raw[i]);
    }
    return z;
}

//explicit instantiation for F=15 (must match MPCController)
template class MPCReference<mpc_config::F>;
