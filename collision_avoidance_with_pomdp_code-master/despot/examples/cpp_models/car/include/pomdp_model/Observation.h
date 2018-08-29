#ifndef DESPOT_OBSERVATION_H
#define DESPOT_OBSERVATION_H


#include <geometry_msgs/Vector3.h>
#include <despot/core/globals.h>
#include "CarState.h"

namespace despot {


class Observation {
private:
    geometry_msgs::Pose car_pose;
    geometry_msgs::Pose pedestrian_position;
    double car_velocity;
    static int _POSITION_DISCR;
    static int _VEL_DISCR;
    static int _ORI_DISCR;

public:
    Observation();
    Observation(geometry_msgs::Pose car_pose,
                geometry_msgs::Pose pedestrian_position,
                double car_velocity);



    /**
     * Encodes the observation by gathering all the information from the current
     * state and encodes it as an OBS_TYPE.
     * @param observation The state to encode the values from.
     */
    static OBS_TYPE EncodeObservation(Observation observation);

    /**
     * Decodes the observation value to the raw values of the observation.
     * @param observation The observation.
     * @return The discretized observation
     */
    static Observation DecodeObservation(OBS_TYPE observation);


    /**
     * Rounds the numerical positions and velocities to a resolution of 0.5m
     * for the positional values and 0.05 m/s for the velocity.
     * @param observation The cars' observation to round the values from.
     */
    static void RoundToResolution(Observation& observation);

    /* ====================== Getters and Setter ============================ */

    const geometry_msgs::Pose get_car_pose() const;

    const geometry_msgs::Pose get_pedestrian_position() const;

    double get_car_velocity() const;


}; // class Observation



} // namespace despot

#endif //DESPOT_OBSERVATION_H
