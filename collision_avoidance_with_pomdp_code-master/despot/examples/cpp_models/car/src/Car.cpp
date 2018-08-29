#include <traffic_models/PedestrianTransitionModel.h>
#include "Car.h"
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/ini_parser.hpp"
#include "pomdp_model/CarBelief.h"
#include "pomdp_model/CarPolicy.h"
#include "../include/traffic_models/CarTransitionModel.h"
#include "pomdp_model/CarScenarioUpperBound.h"
#include <tf/transform_datatypes.h>
#include <pomdp_model/CarParticleLowerBound.h>

#include <utility>

using namespace std;

namespace despot {

/* ==============================================================================
 * Initialize Static Members
 * ==============================================================================*/
geometry_msgs::Pose Car::CAR_START {};
geometry_msgs::Pose Car::PED_START {};
geometry_msgs::Point Car::CAR_GOAL {};
std::vector<geometry_msgs::Point> Car::pedestrian_goals;
double Car::ACCELERATION_VALUE = 0.0;
double Car::MAX_VEL = 0.0;
nav_msgs::Path_<std::allocator<void>>::_poses_type Car::car_path {};

/* ==============================================================================
 * Car Class
 * ==============================================================================*/
Car::Car()
{
    /* Get configuration of the simulation */
    std::string file = "/home/adda_ros/src/code_lina/collision_avoidance_with_pomdp_code-master/despot/examples/cpp_models/car/config/config.ini";
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(file, pt);

    /* Set parameters for the car and the pedestrian. */
    DISTANCE_CAR_TO_GOAL    = pt.get<double>("Car.distance_car_goal");
    DISTANCE_PED_TO_CAR     = pt.get<double>("Car.distance_car_ped") ;
    DESPOT_TERMINAL_RADIUS  = pt.get<double>("Car.terminal_radius")  ;
    ACCELERATION_VALUE      = pt.get<double>("Car.acceleration")     ;
    MAX_VEL                 = pt.get<double>("Car.max_vel")          ;

    /* Gather the rewards from the config file */
    reward_car_near_goal_ = pt.get<double>("Rewards.goal_in_range");
    reward_ped_in_range_  = pt.get<double>("Rewards.ped_in_range");
    reward_acceleration_  = pt.get<double>("Rewards.acceleration");
    reward_deceleration_  = pt.get<double>("Rewards.deceleration");

    ros::Time::init();
}

geometry_msgs::Vector3 Car::Normalize(geometry_msgs::Point vector) {
    double norm = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    geometry_msgs::Vector3 normalized_vector;
    normalized_vector.x = vector.x / norm;
    normalized_vector.y = vector.y / norm;
    normalized_vector.z = vector.z / norm;
    return normalized_vector;
}

bool Car::Step(State &state, double rand_num, int action, double &reward, OBS_TYPE &observation) const
{
    reward = 0;
    observation = 0;
    // Downcast the given state to a CarState to get access to its members
    auto & carState = static_cast<CarState&>(state);

    // Make Transition for Car Model
    CarTransitionModel::MakeTransition(carState, action, CarBelief::_dt);

    // Make Transition for Pedestrian Model
    PedestrianTransitionModel::MakeTransition(carState, action, CarBelief::_dt);

    // Calculate Rewards
    reward += Reward(carState, action);

    // Make observation
    observation = Observation::EncodeObservation
        (Observation(carState.car_pose, carState.ped_pose, carState.car_velocity));

    // Check if terminal state is reached and return
    return IsTerminalState(carState);
}

State * Car::CreateStartState(string type) const
{
    if (type == "DEFAULT")
    {
        CarState* start_state = memory_pool_.Allocate();
        start_state->car_pose       = get_car_start();
        start_state->ped_pose       = get_ped_start();
        start_state->ped_goal       = get_pedestrian_goals()[0];
        start_state->car_velocity   = 0.0;
        start_state->ped_velocity   = 0.5;
        return start_state;
    }
    else
    {
        std::cout << "\033[1;31mFailed to create starting state! Wrong type! Got " << type << "\033[0m\n";
        exit(1);
    }
}

Belief* Car::InitialBelief(const State *start, string type) const
{
    unsigned long num_goals = get_pedestrian_goals().size();
    unsigned long num_particles = CarBelief::num_particles;
    std::vector<State*> particles;
    particles.reserve(num_particles);
    std::vector<geometry_msgs::Point> goal_positions = get_pedestrian_goals();

    int particles_per_goal = num_particles / (num_goals * 2);

    // Have particles with equally distributed goals
    if (type == "UNIFORM")
    {
        for (size_t goal = 0; goal < num_goals; ++goal) {
            for (int i = 0; i < 2; ++i) {
                for (size_t j = 0; j < particles_per_goal; ++j) {
                    CarState *particle = static_cast<CarState *>(CreateStartState("DEFAULT"));
                    particle->ped_goal = goal_positions[goal];
                    particle->weight = 1.0 / num_particles;
                    particle->trust = static_cast<bool>(i);
                    particles.push_back(particle);
                }
            }
        }
        // If particles are missing, fill up with first goal
        auto rest = num_particles % (num_goals * 2);
        for (int i = 0; i < rest; ++i) {
            CarState *particle = static_cast<CarState *>(CreateStartState("DEFAULT"));
            particle->ped_goal = goal_positions[0];
            particle->weight = 1.0 / num_particles;
            particle->trust = true;
            particles.push_back(particle);
        }
    }
    // Choose random goals
    else if (type == "RANDOM")
    {
        for (size_t i = 0; i < num_particles; ++i)
        {
            CarState *particle = static_cast<CarState *>(CreateStartState("DEFAULT"));
            double random_goal_idx = Random::RANDOM.NextInt(0, static_cast<int>(num_goals - 1));
            particle->ped_goal = goal_positions[ random_goal_idx ];
            particle->weight = 1.0 / num_particles;
            particles.push_back(particle);
        }
    }
    else
    {
        std::cout << "Wrong Initial Belief Type! Got " << type << std::endl;
        exit(1);
    }
    assert(particles.size() == num_particles);
    return new CarBelief(this, particles);
}

int Car::NumActions() const
{
    return 3;
}

double Car::GetMaxReward() const
{
    return reward_car_near_goal_;
}

ValuedAction Car::GetMinRewardAction() const
{
    return {ACCELERATE, reward_ped_in_range_};
}

double Car::HighSpeedReward(double current_vel) const
{
    return (current_vel - MAX_VEL)/MAX_VEL;
}


ScenarioLowerBound* Car::CreateScenarioLowerBound(std::string bound_name, std::string particle_bound_name) const
{
    return new CarPolicy(this, CreateParticleLowerBound(particle_bound_name));
}

/* ==============================================================================
 * Helper Functions
 * ==============================================================================*/

bool Car::in_range(geometry_msgs::Point a, geometry_msgs::Point b, double radius)
{
    double distance = sqrt( pow(a.x - b.x, 2) + pow(a.y - b.y, 2) );
    return distance <= radius;
}


/* ==============================================================================
 * Printing Functions
 * ==============================================================================*/
void Car::PrintState(const State &state, std::ostream &out) const {}
void Car::PrintAction(int action, std::ostream &out) const {}
void Car::PrintBelief(const Belief &belief, std::ostream &out) const  {}
void Car::PrintObs(const State &state, OBS_TYPE obs, std::ostream &out) const {}

/* ==============================================================================
 * Memory Management
 * ==============================================================================*/
State* Car::Allocate(int state_id, double weight) const {
    CarState* state = memory_pool_.Allocate();
    state->state_id = state_id;
    state->weight = weight;
    return state;
}

CarState* Car::Copy(const State* particle) const {
    CarState* state = memory_pool_.Allocate();
    *state = *static_cast<const CarState*>(particle);
    state->SetAllocated();
    return state;
}


void Car::Free(State *particle) const {
    memory_pool_.Free(static_cast<CarState*>(particle));
}

int Car::NumActiveParticles() const {
    return memory_pool_.num_allocated();
}

/* ==============================================================================
 * Getters and Setters
 * ============================================================================== */
void Car::set_goal(geometry_msgs::PoseStamped goal) {
    if (goal.pose.position.x == -1 && goal.pose.position.y == -1) {
        default_out << "\033[1;31mGot invalid car goal position!\033[0m\n";
        exit(1);
    }
    Car::CAR_GOAL.x = goal.pose.position.x;
    Car::CAR_GOAL.y = goal.pose.position.y;

    std::cout << "\033[mGot goal! It's " << CAR_GOAL.x << ", "
              << CAR_GOAL.y << "\033[0m" << std::endl;
}

void Car::set_car_start(geometry_msgs::PoseWithCovariance pose) {
    if (pose.pose.position.x == 0 || pose.pose.position.y == 0) {
        default_out << "\033[1;31mGot invalid car position!\033[0m\n";
        exit(1);
    }
    CAR_START.position = pose.pose.position;

    // Get rpy angle of car
    tf::Quaternion quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                              pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    CAR_START.orientation.z = yaw;
    CAR_START.orientation.w = yaw;

    default_out << "Got start position of car: "
                << CAR_START.position.x << ", " << CAR_START.position.y << "\n";
    default_out << "Orientation: " << yaw << "\n";
}

void Car::set_ped_start(double x, double y) {
    if (x == -1 && y == -1) {
        default_out << "\033[1;31mGot invalid pedestrian position!\033[0m\n";
        exit(1);
    }
    Car::PED_START.position.x = x;
    Car::PED_START.position.y = y;
    default_out << "\033[1m Got start position of pedestrian: "
                << Car::PED_START.position.x << ", " << Car::PED_START.position.y << "\033[0m\n";
}

void Car::set_pedestrian_goals(std::vector<geometry_msgs::Point> pedestrian_goals) {
    unsigned long size = pedestrian_goals.size();
    assert(size != 0);

    default_out << "Got all goals:\n";
    for (int i = 0; i < size; ++i) {
        Car::pedestrian_goals.push_back(pedestrian_goals[i]);
        default_out << "Goal no. " << i + 1 << ": \t";
        default_out << Car::pedestrian_goals[i].x << ", " << Car::pedestrian_goals[i].y << std::endl;
    }
}

geometry_msgs::Point Car::get_car_goal() const { return Car::CAR_GOAL; }

geometry_msgs::Pose Car::get_car_start() const { return Car::CAR_START; }

geometry_msgs::Pose Car::get_ped_start() const { return Car::PED_START; }

std::vector<geometry_msgs::Point> Car::get_pedestrian_goals() const {
    return pedestrian_goals;
}

void Car::PrintAction(int action, Printer printer) const
{
    printer.draw_top();
    printer.write("Action: ", printer.BLUE);
    printer.newline();
    switch (action) {
        case 0:
            printer.write("      Accelerating");
            break;
        case 1:
            printer.write("      Holding Velocity");
            break;
        case 2:
            printer.write("      Decelerating");
            break;
        default:break;
    }
    printer.draw_mid();

}

void Car::PrintBelief(const Belief &belief, Printer printer) const
{
    const auto & carBelief = static_cast<const CarBelief&>(belief);
    std::vector<double> distribution = carBelief._goal_distribution;
    std::vector<double> type_distribution = carBelief._type_distribution;

    /* Printing the distribution with little ASCII blocks */
    printer.write("Belief: ", printer.GREEN);
    printer.newline();

    for (int i = 0; i < distribution.size(); ++i) {
        printer.write("      Goal " + std::to_string(i+1) + ": " + to_string_with_precision(distribution[i], 4) + " ");
        for (int j = 0; j < distribution[i]*20; ++j) {
            printer.write("\u2588");
        }
        printer.newline();
    }
    printer.newline();
    printer.write("      Untrusty:\t" + to_string_with_precision(type_distribution[0], 4));
    printer.newline();
    printer.write("      Trusty:\t\t" + to_string_with_precision(type_distribution[1], 4));
    printer.newline();
    printer.draw_mid();
}

void Car::PrintObs(const State &state, OBS_TYPE obs, Printer printer) const
{
    printer.write("Observation: ", printer.RED);
    printer.newline();

    Observation o = Observation::DecodeObservation(obs);
    double car_x = o.get_car_pose().position.x;
    double car_y = o.get_car_pose().position.y;
    double ped_x = o.get_pedestrian_position().position.x;
    double ped_y = o.get_pedestrian_position().position.y;
    double car_v = o.get_car_velocity();
    double car_th = o.get_car_pose().orientation.z;

    printer.write("             Car Position: " + to_string_with_precision(car_x) + ", " + to_string_with_precision(car_y));
    printer.newline();
    printer.write("      Pedestrian Position: " + to_string_with_precision(ped_x) + ", " + to_string_with_precision(ped_y));
    printer.newline();
    printer.write("             Car Velocity: " + to_string_with_precision(car_v));
    printer.newline();
    printer.write("          Car Orientation: " + to_string_with_precision(car_th));
    printer.draw_mid();
}

std::string Car::to_string_with_precision(const double value, const int n) const
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(n) << value;
    return out.str();
}
double Car::Reward(CarState state, int action) const {
    double reward = 0;

    // Reward for safe driving with high velocity
    reward += HighSpeedReward(state.car_velocity);

    // Cost for accelerating / decelerating
    switch (action) {
        case ACCELERATE: {
            reward += reward_acceleration_;
            break;
        }
        case DECELERATE: {
            reward += reward_deceleration_;
            break;
        }
        default: break;
    }

    // Calculate cost for possible pedestrian collision.
    if (in_range(state.car_pose.position, state.ped_pose.position, DISTANCE_PED_TO_CAR)) {
        reward += reward_ped_in_range_;
    }

    // Calculate reward for car getting closer to the goal.
    if (in_range(state.car_pose.position, get_car_goal(), DISTANCE_CAR_TO_GOAL)) {
        reward += reward_car_near_goal_;
    }

    if (in_range(state.car_pose.position, state.ped_pose.position, DESPOT_TERMINAL_RADIUS))
        reward = 0;

    return reward;
}
bool Car::IsTerminalState(CarState state) const {
    return in_range(state.car_pose.position, state.ped_pose.position, DISTANCE_PED_TO_CAR)
    || in_range(state.car_pose.position, get_car_goal(), DISTANCE_CAR_TO_GOAL);
}


ScenarioUpperBound *Car::CreateScenarioUpperBound(string name, string particle_bound_name) const {
    return new CarScenarioUpperBound(this);
}

double Car::GetDistance(geometry_msgs::Point start, geometry_msgs::Point destination)
{
    return sqrt(pow(destination.x - start.x, 2) + pow(destination.y - start.y, 2));

}

void Car::set_path(nav_msgs::Path_<std::allocator<void>>::_poses_type vector)
{
    car_path = std::move(vector);
    std::cout << "Got car path! " << car_path.size() << std::endl;
}


} // namespace despot
