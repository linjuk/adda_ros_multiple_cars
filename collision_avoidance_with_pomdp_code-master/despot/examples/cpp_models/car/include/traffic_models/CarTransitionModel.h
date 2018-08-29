
#ifndef DESPOT_CAR_TRANSITION_MODEL_H
#define DESPOT_CAR_TRANSITION_MODEL_H

#include "transition_model.h"

namespace despot {

class CarTransitionModel {
public:
    static void MakeTransition(CarState &state, int action, double dt);

};

} // namespace despot


#endif //DESPOT_CAR_TRANSITION_MODEL_H
