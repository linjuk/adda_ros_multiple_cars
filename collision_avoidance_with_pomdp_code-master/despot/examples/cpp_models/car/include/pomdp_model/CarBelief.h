#ifndef DESPOT_CAR_BELIEF_H
#define DESPOT_CAR_BELIEF_H

#include <ros/ros.h>
#include "despot/core/belief.h"
#include "Car.h"
#include <fstream>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace despot {

/**
 * This class represents a Belief for the POMDP. It provides an Update function
 * for updating the particles and a Sample function for sampling a given
 * amount of particles from the model.
 * The Belief is a probability distribution over all predefined goals.
 */
class Car;
class CarBelief : public ParticleBelief {
protected:
    const Car*          _car_model;

private:
    unsigned long       _num_goals;
    std::ofstream       _file;
    ros::Time           _last_time;
    ros::NodeHandle     nodeHandle;
    ros::Subscriber     pos_sub;
    ros::Subscriber     active_goal_sub;
    ros::Subscriber     vel_sub;
    ros::Subscriber     car_pos_sub;

public:
    static double _dt;
    static unsigned int num_particles;
    ros::NodeHandle nhh;
    ros::ServiceClient client;
    explicit CarBelief(const Car *model, std::vector<State*> particles);
    void AddDistributionToFile(std::vector<double> distribution);

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;


    /**
     * Samples a given amount of particles from the particle vector and returns them.
     * @param num The amount of particles (States) to sample.
     * @return A vector of particles.
     */
    std::vector<State*> Sample(int num) const override;

    /**
     * Updates the Belief given the action and the resulting observation.
     * @param action The action made in state s.
     * @param observation The observation of the successor state s'.
     */
    void Update(int action, OBS_TYPE observation) override;

    /**
     * This function will return the current probability distribution
     * over all goals. Every entry represents the distribution over
     * the goal respectively.
     * @return Vector of distributions, namely Belief over goals.
     */
    std::vector<double> ComputeGoalDistribution() const;

    std::vector<std::vector<double>> distribution()const {
        return _current_distribution;
    }


    std::vector<double> ComputeTypeDistribution() const;

    std::vector<std::vector<double>> _current_distribution;
    std::vector<double> _goal_distribution;
    std::vector<double> _type_distribution;
private:
    void InitializeSubscriber();
    void AddRealGoalToFile();
    void AddVelocityToFile();
    void AddPedPositionToFile();
    void AddCarPosToFile();

    std::vector<std::vector<double>> ComputeFullDistribution() const;

    void AddTypeToFile();
};
} // namespace despot

#endif //DESPOT_CAR_BELIEF_H
