
#include "pomdp_model/CarParticleLowerBound.h"

namespace despot {

CarParticleLowerBound::CarParticleLowerBound(const DSPOMDP *model)
    : ParticleLowerBound(model),
      _car(static_cast<const Car *>(model))
{
}

ValuedAction CarParticleLowerBound::Value(const std::vector<State *> &particles) const
{
    const auto &s = static_cast<const CarState&>(*particles[0]);
    return {
        _car->DECELERATE,
        State::Weight(particles) * (_car->reward_ped_in_range_ / (1 - Globals::Discount()))
    };
}

} // namespace despot
