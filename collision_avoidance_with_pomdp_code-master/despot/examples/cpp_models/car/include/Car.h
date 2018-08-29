#ifndef CAR_H
#define CAR_H

#include <iostream>
#include <despot/core/pomdp.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "pomdp_model/CarState.h"
#include "util/Printer.h"
#include <vector>


namespace despot {


/* ==============================================================================
 * Car Class
 * ==============================================================================*/
class Car : public DSPOMDP {
protected:
    static geometry_msgs::Point CAR_GOAL;
    static geometry_msgs::Pose CAR_START;
    static geometry_msgs::Pose PED_START;

private:

    static std::vector<geometry_msgs::Point> pedestrian_goals;
    mutable MemoryPool<CarState> memory_pool_;


public:
    double DESPOT_TERMINAL_RADIUS;
    double DISTANCE_CAR_TO_GOAL;
    double DISTANCE_PED_TO_CAR;
    static double ACCELERATION_VALUE;
    static double MAX_VEL;

    static nav_msgs::Path_<std::allocator<void>>::_poses_type car_path;
    /**
     * Constructor will initialize the car parameters such the maximum velocity of
     * the car and the acceleration. Also the distances from the car to the goal
     * and the pedestrian for the reward model are parsed here.
     * All values will be read from a config file "config.ini".
     */
    Car();


    /**
     * Enumerations for the actions that the car can execute.
     */
    enum Actions { ACCELERATE, HOLD, DECELERATE };


    /**
     * Returns the number of actions in the POMDP.
     */
    int NumActions() const override;


    /**
     * Simulates one step of the environment given the current state and the action
     * to execute for the car. The reward and the observation resulting from that
     * execution will be written to the corresponding variables. The given state
     * will also be the successor state afterwards.
     * @param state Current state of the environment.
     * @param rand_num Random number
     * @param action The action to execute
     * @param reward The reward resulting from the action
     * @param observation The observation after executing the action
     * @return              A boolean whether the state results in a terminal state.
     *                      This happens when the car reaches the goal to a certain
     *                      range TERMINAL_RANGE.
     */
    bool Step(State& state, double rand_num, int action, double& reward, OBS_TYPE& observation)
    const override;


    /**
     * Creates the initial state for the model.
     * @param type Define the type of the initial distribution, e.g. UNIFORM.
     * @return A pointer to the starting state.
     */
    State* CreateStartState(std::string type = "DEFAULT") const override;


    /**
     * Creates the initial belief for a given state. This given state is optional.
     * @param start State to draw initial belief from.
     * @param type Type of initial belief
     * @return A ParticleBelief object.
     */
    Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const override;


    /**
     * Calculates the probability of observing obs when executing action and resulting in state.
     * @param obs The observation after taking the action
     * @param state The resulting state after taking the action.
     * @param action The action to execute.
     * @return The probability of observing obs when executing action and resulting in state.
     */
    double ObsProb(OBS_TYPE obs, const State& state, int action) const override { return 0; }

    ScenarioLowerBound*
    CreateScenarioLowerBound(std::string bound_name = "DEFAULT",
                             std::string particle_bound_name = "DEFAULT") const override;

    ScenarioUpperBound *CreateScenarioUpperBound(std::string name, std::string particle_bound_name) const override;

    /**
     * Computes the distance between two Points.
     * @param start
     * @param destination
     * @return
     */
    static double GetDistance(geometry_msgs::Point start, geometry_msgs::Point destination);

    /* ============================================================================================= */
    /* ==================== Printing functions for debugging ======================================= */
    /* ============================================================================================= */

    void PrintObs(const State& state, OBS_TYPE obs, Printer printer) const;
    void PrintBelief(const Belief& belief, Printer printer) const;
    void PrintAction(int action, Printer printer) const;

    /* Not used print functions */
    void PrintState(const State& state, std::ostream& out = std::cout) const override;
    void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override;
    void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override;
    void PrintAction(int action, std::ostream& out = std::cout) const override;


    /* ============================================================================================= */
    /* ================================= Rewards =================================================== */
    /* ============================================================================================= */

    ValuedAction GetMinRewardAction() const override;
    double GetMaxReward() const override;
    double HighSpeedReward(double current_vel) const;
    double Reward(CarState state, int action) const;
    double reward_ped_in_range_;
    double reward_car_near_goal_;
    double reward_acceleration_;
    double reward_deceleration_;


    /* ============================================================================================= */
    /* ================================= Helper Functions ========================================== */
    /* ============================================================================================= */

    /**
     * Checks whether point a lies in a radius to point b.
     * @param a : Array of (x,y) position of first point
     * @param b : Array of (x,y) position of second point
     * @param radius : Radius for a
     * @return True if b lies in area with radius to a.
     */
    static bool in_range(geometry_msgs::Point a, geometry_msgs::Point b, double radius);


    /**
     * Converts a given floating point value with precision n and returns it
     * as a string.
     * @param value Value to be precised.
     * @param n Precision value
     * @return Value as string
     */
    std::string to_string_with_precision(double value, int n = 1) const;


    /* ============================================================================================= */
    /* ================================= Memory Management ========================================= */
    /* ============================================================================================= */

    State* Allocate(int state_id, double weight) const override;
    CarState * Copy(const State* particle) const override;
    void Free(State* particle) const override;
    int NumActiveParticles() const override;


    /* ============================================================================================= */
    /* ================================= Getters and Setters ======================================= */
    /* ============================================================================================= */

    /**
     * Sets the goal of the car from rViz for this car object.
     * @param goal The goal position as a class geometry_msgs::PoseStamped.
     */
    static void set_goal(geometry_msgs::PoseStamped goal);


    /**
     * Gets the starting position of the car from rViz and sets the
     * corresponding variable.
     * @param x The x value from the car's starting position.
     * @param y The y value from the car's starting position.
     */
    static void set_car_start(geometry_msgs::PoseWithCovariance pose);


    /**
     * Gets the starting position of the pedestrian from rViz and
     * sets the corresponding variable.
     * @param x The x value from the pedestrian's starting position.
     * @param y The y value from the pedestrian's starting position.
     */
    static void set_ped_start(double x, double y);


    /**
     * Gets all the goals from the environment and pushes the positions
     * into the corresponding class variable.
     * @param pedestrian_goals All the goal positions from the environment.
     */
    static void set_pedestrian_goals(std::vector<geometry_msgs::Point> pedestrian_goals);


    geometry_msgs::Point get_car_goal() const;
    geometry_msgs::Pose get_car_start() const;
    geometry_msgs::Pose get_ped_start() const;
    std::vector<geometry_msgs::Point> get_pedestrian_goals() const;

    static geometry_msgs::Vector3 Normalize(geometry_msgs::Point vector);
    bool IsTerminalState(CarState state) const;

    static void set_path(nav_msgs::Path_<std::allocator<void>>::_poses_type vector);

};
}
#endif //CAR_H

