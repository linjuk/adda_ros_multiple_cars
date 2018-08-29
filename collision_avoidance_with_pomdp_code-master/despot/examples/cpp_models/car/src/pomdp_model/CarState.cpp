#include "pomdp_model/CarState.h"
#include <ostream>

namespace despot {


CarState::CarState(geometry_msgs::Pose car_position,
                   geometry_msgs::Pose ped_position,
                   geometry_msgs::Point ped_goal, double car_velocity, double ped_velocity, bool trust)
        : car_pose(car_position),
          ped_pose(ped_position),
          ped_goal(ped_goal),
          car_velocity(car_velocity),
          ped_velocity(ped_velocity),
          trust(trust)
{
}

std::string CarState::text() const
{
    std::ostringstream stream;
    stream << "\nCar Position:\t" << this->car_pose.position.x << ", " << this->car_pose.position.y << "\n";
    stream << "Ped Position:\t" << this->ped_pose.position.x << ", " << this->ped_pose.position.y << "\n";
    stream << "Ped Goal:\t\t" << this->ped_goal.x << ", " << this->ped_goal.y << "\n";
    stream << "Car Velocity:\t" << this->car_velocity << "\n";
    stream << "Ped Velocity:\t" << this->ped_velocity << "\n";
    stream << "Car Orientation:\t" << this->car_pose.orientation.z << "\n";
    stream << "Trust:\t" << this->trust << "\n";

    return stream.str();

}

}
