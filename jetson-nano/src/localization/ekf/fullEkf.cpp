#include "localization/ekf/fullEkf.hpp"

#include <cmath>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

FullEkf::FullEkf(const FullEkfNoise& noise) : m_noise(noise) {}

void FullEkf::predict(const PoseIncrement& inc) {
    // TODO (#186): drainQueue() first, so measurements apply before this cycle's motion
    // TODO (#186): skip an increment with a NaN delta or covariance
    // TODO (#186): lock m_stateMutex for the rest
    // TODO (#186): F = se2::composeJacA(m_x, inc.delta), G = se2::composeJacB(m_x), both at the old m_x
    // TODO (#186): m_x = se2::compose(m_x, inc.delta)
    // TODO (#186): m_P = F m_P Fᵀ + G inc.cov Gᵀ + m_noise.qExtra, then symmetrize
    // TODO (#186): m_tUs = inc.t1Us
}

void FullEkf::submit(const PoseMeasurement& m) {
    // TODO (#186): lock m_pendingMutex and push m onto m_pending
}

StampedPose FullEkf::latest() const {
    // TODO (#186): lock m_stateMutex, return StampedPose{m_tUs, m_x, m_P}
    return StampedPose{};
}

UpdateResult FullEkf::lastUpdate() const {
    // TODO (#186): lock m_stateMutex, return m_lastUpdate
    return UpdateResult{};
}

UpdateResult FullEkf::update(const PoseMeasurement& m) {
    // TODO (#186): reject a measurement with a NaN pose or covariance
    // TODO (#186): ν = z − x, with ν(2) = utils::wrapAngle(z.theta − x.theta)
    // TODO (#186): S = m_P + m.cov, d² = νᵀ S⁻¹ ν (solve with S.ldlt(), don't invert)
    // TODO (#186): d² > m_noise.gate -> rejected, leave m_x and m_P alone
    // TODO (#186): K = m_P S⁻¹, x += K ν, wrap θ
    // TODO (#186): Joseph form m_P = (I − K) m_P (I − K)ᵀ + K R Kᵀ, then symmetrize
    // TODO (#186): store the result in m_lastUpdate, every update records d² for #192
    // TODO (#186): delayed measurements (tUs in the past) are applied as if current here, #187 replays them at the right time
    return UpdateResult{};
}

void FullEkf::drainQueue() {
    // TODO (#186): swap m_pending into a local vector under m_pendingMutex, so submit() isn't blocked while updating
    // TODO (#186): sort by tUs, then update() each under m_stateMutex
}
