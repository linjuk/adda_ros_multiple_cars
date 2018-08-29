#ifndef DESPOT_CAR_STATE_H
#define DESPOT_CAR_STATE_H

#include <despot/core/pomdp.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

namespace despot {

/**
 * This class represents a state from the POMDP model to handle transitions from
 * one state to another state.
 */
class CarState : public State {
public:
    geometry_msgs::Pose car_pose;
    geometry_msgs::Pose ped_pose;
    geometry_msgs::Point ped_goal;
    double car_velocity {};
    double ped_velocity {};
    bool trust = true;

    CarState() = default;

    CarState(geometry_msgs::Pose car_position,
             geometry_msgs::Pose ped_position,
             geometry_msgs::Point ped_goal,
             double car_velocity,
             double ped_velocity,
             bool trust);

    std::string text() const override;

};

} // namespace despot

#endif //DESPOT_CAR_STATE_H
