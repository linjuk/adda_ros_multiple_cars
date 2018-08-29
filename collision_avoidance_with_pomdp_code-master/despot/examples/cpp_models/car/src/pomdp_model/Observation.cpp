#include <Car.h>
#include "pomdp_model/Observation.h"

namespace despot {

int Observation::_POSITION_DISCR = 5;
int Observation::_VEL_DISCR = 10;
int Observation::_ORI_DISCR = 10;


Observation::Observation() = default;

despot::Observation::Observation(geometry_msgs::Pose car_pose,
                                 geometry_msgs::Pose pedestrian_position,
                                 double car_velocity) :
                    car_pose(car_pose),
                    pedestrian_position(pedestrian_position),
                    car_velocity(car_velocity)
{

}


OBS_TYPE Observation::EncodeObservation(Observation observation)
{
    /* Round to 0.5 and 0.1 for position and velocity. */
    RoundToResolution(observation);

    /* Multiply position by 2 and velocity by 10 to get integer values. */
    uint64_t car_x   = static_cast<uint64_t>(observation.car_pose.position.x * _POSITION_DISCR);
    uint64_t car_y   = static_cast<uint64_t>(observation.car_pose.position.y * _POSITION_DISCR);
    uint64_t ped_x   = static_cast<uint64_t>(observation.pedestrian_position.position.x * _POSITION_DISCR);
    uint64_t ped_y   = static_cast<uint64_t>(observation.pedestrian_position.position.y * _POSITION_DISCR);
    uint64_t car_vel = static_cast<uint64_t>(observation.car_velocity * _VEL_DISCR);
    uint64_t car_th  = static_cast<uint64_t>(observation.car_pose.orientation.z * _ORI_DISCR);


    /* Encode all the values 8 bit-wise to one OBS_TYPE. */
    return static_cast<OBS_TYPE>(car_x | (car_y << 8) | (ped_x << 16)
                                 | (ped_y << 24) | (car_vel << 32) | (car_th << 40));
}


Observation Observation::DecodeObservation(OBS_TYPE o)
{
    /* Read observation and decode it back to their raw values. */
    double car_x = (o & 0xFF);
    double car_y = ((o >> 8) & 0xFF);
    double ped_x = ((o >> 16) & 0xFF);
    double ped_y = ((o >> 24) & 0xFF);
    double car_v = ((o >> 32) & 0xFF);
    double car_th = ((o >> 40) & 0xFF);
    car_x /= _POSITION_DISCR;
    car_y /= _POSITION_DISCR;
    ped_x /= _POSITION_DISCR;
    ped_y /= _POSITION_DISCR;
    car_v /= _VEL_DISCR;
    car_th /= _ORI_DISCR;

    geometry_msgs::Pose car_pose;
    car_pose.position.x = car_x;
    car_pose.position.y = car_y;
    car_pose.orientation.z = car_th;
    car_pose.orientation.w = car_th;

    geometry_msgs::Pose pedestrian_position;
    pedestrian_position.position.x = ped_x;
    pedestrian_position.position.y = ped_y;

    return {car_pose, pedestrian_position, car_v};
}


void Observation::RoundToResolution(Observation& observation)
{
    observation.car_pose.position.x = ceil(_POSITION_DISCR*observation.car_pose.position.x)/_POSITION_DISCR;
    observation.car_pose.position.y = ceil(_POSITION_DISCR*observation.car_pose.position.y)/_POSITION_DISCR;
    observation.pedestrian_position.position.x = ceil(_POSITION_DISCR*observation.pedestrian_position.position.x)/_POSITION_DISCR;
    observation.pedestrian_position.position.y = ceil(_POSITION_DISCR*observation.pedestrian_position.position.y)/_POSITION_DISCR;
    observation.car_velocity = ceil(_VEL_DISCR*observation.car_velocity)/_VEL_DISCR;
    observation.car_pose.orientation.z = ceil(_ORI_DISCR*observation.car_pose.orientation.z)/_ORI_DISCR;
}


/* ====================== Getters and Setter ============================ */

const geometry_msgs::Pose Observation::get_car_pose() const {
    return car_pose;
}

const geometry_msgs::Pose Observation::get_pedestrian_position() const {
    return pedestrian_position;
}

double Observation::get_car_velocity() const {
    return car_velocity;
}

}
