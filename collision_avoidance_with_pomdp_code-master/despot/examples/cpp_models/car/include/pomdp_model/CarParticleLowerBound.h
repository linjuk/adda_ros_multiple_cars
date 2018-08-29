#pragma once

#include <iostream>
#include <string>
#include <despot/core/lower_bound.h>
#include <despot/core/pomdp.h>
#include <despot/core/belief.h>
#include "Car.h"

namespace despot {


class CarParticleLowerBound : public ParticleLowerBound {
protected:
    const Car* _car;

public:
    CarParticleLowerBound(const DSPOMDP *model);

    ValuedAction Value(const std::vector<State *> &particles) const override;

};


}
