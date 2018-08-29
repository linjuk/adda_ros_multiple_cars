#ifndef DESPOT_CAR_POLICY_H
#define DESPOT_CAR_POLICY_H

#include "despot/core/policy.h"
#include "Car.h"
#include "Observation.h"
#include "ostream"

namespace despot {

/**
 * This class represents a default policy for the DESPOT to execute to get a lower bound in the tree.
 */
class CarPolicy : public Policy {
private:
    const Car* _model;

public:
    CarPolicy(const DSPOMDP *model, ParticleLowerBound *particle_lower_bound);

    /**
     * Computes the action for the DESPOT to execute under a default policy.
     * @param particles The sampled particle vector.
     * @param streams Stream of random numbers, drawn from [0,1].
     * @param history The history of action-observation pairs.
     * @return An action based on the policy.
     */
    int Action(const std::vector<State*>& particles, RandomStreams& streams, History& history) const override;

};
}


#endif //DESPOT_CAR_POLICY_H
