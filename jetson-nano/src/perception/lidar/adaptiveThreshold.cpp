#include "perception/lidar/adaptiveThreshold.hpp"

#include "mpc/referenceRollout.hpp"  

#include <cmath>

// TODO: ΔT = tInit⁻¹ ∘ tResult; δ = 2*r_max*sin(|Δθ|/2) + ‖Δt‖
// TODO: if δ > δ_min: sumSq += δ², count++
// TODO: sigma(): sqrt(sumSq / count)
// TODO: constructor and reset(): sumSq = (τ_0/3)², count = 1