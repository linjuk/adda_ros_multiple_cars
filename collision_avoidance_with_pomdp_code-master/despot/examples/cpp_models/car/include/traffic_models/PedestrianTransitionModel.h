#pragma once

#include "transition_model.h"

namespace despot
{

class PedestrianTransitionModel {
public:
    static std::default_random_engine generator;

    static void MakeTransition(CarState &state, int action, double dt);
};

} // namespace despot

