#include <Car.h>
#include "traffic_models/CarTransitionModel.h"

namespace despot {


void CarTransitionModel::MakeTransition(CarState &state, int action, double dt)
{
    if (action == Car::ACCELERATE)
    {
        if (state.car_velocity <= Car::MAX_VEL - Car::ACCELERATION_VALUE)
        {
            state.car_velocity += Car::ACCELERATION_VALUE;
        }
    }
    else if (action == Car::DECELERATE)
    {
        if (state.car_velocity >= 0.0 + Car::ACCELERATION_VALUE)
        {
            state.car_velocity -= Car::ACCELERATION_VALUE;
        }
    }

    state.car_pose.position.x += state.car_velocity * cos(state.car_pose.orientation.z) * dt;
    state.car_pose.position.y += state.car_velocity * sin(state.car_pose.orientation.z) * dt;
}

} // namespace despot