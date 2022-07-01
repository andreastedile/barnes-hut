#include "exact_simulation_step.h"

#include <utility>  // move

namespace bh {

ExactSimulationStep::ExactSimulationStep(std::vector<Body> bodies, const AlignedBox2d& bbox)
    : SimulationStep(std::move(bodies), bbox) {}

}  // namespace bh