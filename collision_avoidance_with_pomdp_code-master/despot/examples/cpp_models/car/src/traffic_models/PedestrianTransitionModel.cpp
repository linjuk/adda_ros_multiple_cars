#include <Car.h>
#include "traffic_models/PedestrianTransitionModel.h"
#include "../../include/util/eigenmvn.h"
#include "Eigen/Core"

namespace despot {

std::default_random_engine PedestrianTransitionModel::generator {};


void PedestrianTransitionModel::MakeTransition(despot::CarState &state, int action, double dt)
{
    // Get the direction towards the goal of the pedestrian
    geometry_msgs::Point goal_position = state.ped_goal;
    geometry_msgs::Point direction;
    direction.x = goal_position.x - state.ped_pose.position.x;
    direction.y = goal_position.y - state.ped_pose.position.y;

    geometry_msgs::Vector3 normalized_direction = Car::Normalize(direction);

    // Prepare rng
    double mean = 0.0;
    double stddev{};
    if (state.trust)
        stddev = 0.1;
    else
        stddev = 0.5;
    std::normal_distribution<double> dist(mean, stddev);

    // Update the position
    state.ped_pose.position.x += (state.ped_velocity * normalized_direction.x * dt) + dist(generator);
    state.ped_pose.position.y += (state.ped_velocity * normalized_direction.y * dt) + dist(generator);
}

} // namespace despot
