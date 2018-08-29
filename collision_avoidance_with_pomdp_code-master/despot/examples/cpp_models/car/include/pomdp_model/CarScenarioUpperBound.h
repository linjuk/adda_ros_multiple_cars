#pragma once

#include <despot/core/upper_bound.h>
#include <Car.h>

namespace despot {


class CarScenarioUpperBound : public ScenarioUpperBound{
protected:
    const Car* _model;
public:
    explicit CarScenarioUpperBound(const Car* model);

    ~CarScenarioUpperBound() override;

    double Value(const std::vector<State *> &particles, RandomStreams &streams, History &history) const override;
};

}

