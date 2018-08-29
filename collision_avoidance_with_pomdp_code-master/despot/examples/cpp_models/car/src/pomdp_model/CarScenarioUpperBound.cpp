#include "pomdp_model/CarScenarioUpperBound.h"


namespace despot {

CarScenarioUpperBound::CarScenarioUpperBound(const Car* model)
    : _model(model)
{
}

CarScenarioUpperBound::~CarScenarioUpperBound() = default;

double CarScenarioUpperBound::Value(const std::vector<State *> &particles, RandomStreams &streams, History &history) const
{
    double total_value = 0;
    for (auto& particle : particles)
    {
        auto &state = static_cast<CarState&>(*(particle));
        double value = 0;
        double dist = _model->GetDistance(state.ped_pose.position, _model->get_car_goal());
        auto time_steps_to_goal = static_cast<int>(dist / _model->MAX_VEL);

        // In pedestrian range
        value += (_model->in_range(state.car_pose.position, state.ped_pose.position, _model->DISTANCE_PED_TO_CAR) ?
            _model->reward_ped_in_range_ : _model->reward_car_near_goal_ * pow(Globals::Discount(), time_steps_to_goal));

        total_value += state.weight * value;
    }
    return total_value;
}


}
