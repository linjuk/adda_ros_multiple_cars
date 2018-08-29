#include "pomdp_model/CarPolicy.h"
#include "util/ros_utilities.h"
#include "traffic_models/CarTransitionModel.h"

namespace despot {


CarPolicy::CarPolicy(const DSPOMDP *model, ParticleLowerBound *particle_lower_bound) :
    Policy(model, particle_lower_bound),
    _model((Car *) model)
{
}


int CarPolicy::Action(const std::vector<State *> &particles, RandomStreams &streams, History &history) const
{
    if (history.Size() < 1) return _model->HOLD;

    Observation last_observation = Observation::DecodeObservation(history.LastObservation());
    geometry_msgs::Point car_position = last_observation.get_car_pose().position;
    geometry_msgs::Point ped_position = last_observation.get_pedestrian_position().position;

    if (_model->in_range(car_position, ped_position, _model->DISTANCE_PED_TO_CAR)) {
        return _model->DECELERATE;
    } else if (_model->in_range(car_position, _model->get_car_goal(), _model->DISTANCE_CAR_TO_GOAL)) {
        return _model->ACCELERATE;
    }
    else return _model->HOLD;
}

} // namespace despot
