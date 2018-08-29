/**
 * @author Albert Schotschneider
 * @version 1.0
 */
#ifndef DESPOT_MODEL_H
#define DESPOT_MODEL_H

#include "pomdp_model/CarState.h"

namespace despot {

/**
 * Interface for representing a model in the system.
 */
class TransitionModel {
public:
    virtual void MakeTransition(CarState& state, int action, double dt) = 0;
};

} // namespace despot

#endif //DESPOT_MODEL_H
