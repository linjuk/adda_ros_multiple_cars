#include <pomdp_model/Observation.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "pomdp_model/CarBelief.h"
#include "car_model/ProbObservedPosition.h"
#include "util/helper_functions.h"



namespace despot {

unsigned int CarBelief::num_particles = 6000;
geometry_msgs::Vector3 active_goal{};
geometry_msgs::Point pedestrian_position{};
double current_velocity = 0.0f;
double CarBelief::_dt = 1.0;
geometry_msgs::Point car_pos{};
std::vector<double> distance_to_pedestrian{};

void cb_get_active_goal(const geometry_msgs::Vector3Ptr &msg)
{
    active_goal = *msg;
}

void cb_get_current_velocity(const nav_msgs::Odometry &msg)
{
    current_velocity = msg.twist.twist.linear.x;
}

void cb_get_pedestrian_position(const nav_msgs::Odometry &msg)
{
    pedestrian_position = msg.pose.pose.position;
}

void cb_get_car_position(const nav_msgs::Odometry &msg)
{
    car_pos = msg.pose.pose.position;
}

CarBelief::CarBelief(const Car *model, std::vector<State *> particles) :
    ParticleBelief(particles, model),
    _car_model(model),
    _num_goals(_car_model->get_pedestrian_goals().size()),
    _current_distribution(ComputeFullDistribution()),
    _goal_distribution(ComputeGoalDistribution()),
    _type_distribution(ComputeTypeDistribution())
{
    ros::NodeHandle nh("");
    nodeHandle = nh;
    client = nh.serviceClient<car_model::ProbObservedPosition>("/prob_obs_pos", true);
    if (!client.exists())
    {
        std::cout << "\033[1;31mCouldn't connect to ProbObsPos Service !\033[0m\n";
        exit(1);
    }
    else
    {
        std::cout << "\n\033[1;32mConnected to ProbObsPos Service !\033[0m\n";
    }

    // Delete content of old data file and close it.
    _file.open("/home/albert/data", std::ios::trunc);
    _file.close();
    _file.open("/home/albert/real_goal", std::ios::trunc);
    _file.close();
    _file.open("/home/albert/velocity", std::ios::trunc);
    _file.close();
    _file.open("/home/albert/ped_pos", std::ios::trunc);
    _file.close();
    _file.open("/home/albert/car_pos", std::ios::trunc);
    _file.close();
    _file.open("/home/albert/type", std::ios::trunc);
    _file.close();

    InitializeSubscriber();
    _last_time = ros::Time::now();
}


void CarBelief::Update(int action, OBS_TYPE observation)
{
    ros::spinOnce();
    // From the observation, get the position of the pedestrian.
    Observation decoded_observation = Observation::DecodeObservation(observation);
    geometry_msgs::Pose current_observed_ped_position = decoded_observation.get_pedestrian_position();

    // Get the previous position of the pedestrian if existent.
    geometry_msgs::Pose last_observed_position;
    if (history_.Size() > 0)
    {
        Observation old_observation_decoded = Observation::DecodeObservation(history_.LastObservation());
        last_observed_position = old_observation_decoded.get_pedestrian_position();
    }
    else
    {
        // Use the current position as the old position because there were no change in position.
        last_observed_position.position.x = current_observed_ped_position.position.x;
        last_observed_position.position.y = current_observed_ped_position.position.y;
    }

    // Write everything to files
    AddDistributionToFile(_goal_distribution);
    AddRealGoalToFile();
    AddVelocityToFile();
    AddPedPositionToFile();
    AddCarPosToFile();
    AddTypeToFile();

    // Add action-observation pair to the history
    history_.Add(action, observation);

    // Reserve Memory for new Particle Vector
    std::vector<State *> updated_particles;
    updated_particles.reserve(particles_.size());

    ros::Time current_time = ros::Time::now();
    _dt = (current_time - _last_time).toSec();

    // Calculate the velocity vector based on the previous position and the current position.
    geometry_msgs::Vector3 velocity;
    velocity.x = (current_observed_ped_position.position.x - last_observed_position.position.x) / _dt;
    velocity.y = (current_observed_ped_position.position.y - last_observed_position.position.y) / _dt;

    double avg_vel = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2));
    std::cout << "Calc. Ped. Vel. from DESPOT: " << avg_vel << std::endl;

    std::vector<geometry_msgs::Point> all_goals = _car_model->get_pedestrian_goals();

    // ****************************************************************************************************************
    // UPDATE BELIEF OVER GOALS
    // ****************************************************************************************************************

    // Prepare variables for calling the ProbObsPos server
    car_model::ProbObservedPosition srv{};
    srv.request.last_observed_position = last_observed_position;
    srv.request.observed_position = current_observed_ped_position;
    srv.request.velocity = avg_vel;
    srv.request.dt = _dt;

    // Get the current Probability Distribution over all goals.
    std::vector<std::vector<double>> current_distribution = _current_distribution;

    // Also make a vector for storing the probabilities themselves.
    std::vector<std::vector<double>> probabilities(_num_goals);
    for (int i = 0; i < _num_goals; ++i)
    {
        probabilities[i] = {0, 0};
    }

    // Calculate the normalization factor of the belief update equation.
    double normalization_factor = 0;
    for (int goal_idx = 0; goal_idx < _num_goals; ++goal_idx)
    {
        srv.request.goal = all_goals[goal_idx];
        for (int i = 0; i < 2; ++i)
        {
            srv.request.trust = static_cast<unsigned char>(i);
            if (client.call(srv))
            {
                // Get the probability
                double prob = srv.response.probability;
                // Save the current pedestrian probability.
                probabilities[goal_idx][i] = prob;
                // Calculate the normalization factor
                normalization_factor += (prob * _current_distribution[goal_idx][i]);
            }
            else
            {
                std::cout << "\033[31m Could not call ProbObsPos Server!\033[0m\n";
                exit(1);
            }
        }
    }

    // Calculate the new belief. First create a vector for storing the new belief so that the old
    // belief doesn't get overwritten.
    std::vector<std::vector<double>> new_distribution(_num_goals);
    for (int k = 0; k < _num_goals; ++k)
    {
        new_distribution[k] = {0, 0};
    }

    for (int goal = 0; goal < _num_goals; ++goal)
    {
        for (int i = 0; i < 2; ++i)
        {
            new_distribution[goal][i] =
                (probabilities[goal][i] * _current_distribution[goal][i]) / normalization_factor;
        }
    }

    // Marginalize over goals and types. First, free the vector.
    for (int i = 0; i < _num_goals; ++i)
    {
        _goal_distribution[i] = 0;
        _type_distribution[i] = 0;
    }

    for (int g = 0; g < _num_goals; ++g)
    {
        for (int i = 0; i < 2; ++i)
        {
            _goal_distribution[g] += new_distribution[g][i];
        }
    }

    for (int i = 0; i < 2; ++i)
    {
        for (int g = 0; g < _num_goals; ++g)
        {
            _type_distribution[i] += new_distribution[g][i];
        }
    }

    // Make sure the new distribution is never 0
    for (auto &d : new_distribution)
    {
        for (auto &dd : d)
        {
            if (dd < 0.01)
            {
                dd = 0.01;
            }
        }
    }

    //////////////////////////////////
    for (int i = 0; i < _num_goals; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            std::cout << new_distribution[i][j] << "   ";
        }
        std::cout << std::endl;
    }
    ////////////////////////////////

    // ****************************************************************************************************************
    // UPDATING PARTICLE VECTOR
    // ****************************************************************************************************************

    std::vector<int> particles_per_prob;
    for (int i = 0; i < _num_goals; ++i) {
        for (int j = 0; j < 2; ++j) {
            particles_per_prob.push_back(static_cast<int &&>(new_distribution[i][j] * num_particles_));
        }
    }

    std::cout << "NUMBER OF PARTICLES:" << std::endl;
    std::cout << "G1, T1: " << particles_per_prob[0] << std::endl;
    std::cout << "G1, T2: " << particles_per_prob[1] << std::endl;
    std::cout << "G2, T1: " << particles_per_prob[2] << std::endl;
    std::cout << "G2, T2: " << particles_per_prob[3] << std::endl;

    // Calculate the number of particles for every goal.
    std::vector<int> particles_per_goal_vector(_num_goals);
    int total_but_last = 0;
    for (int i = 0; i < _num_goals; ++i)
    {
        if (i < _num_goals - 1)
        {
            particles_per_goal_vector[i] = static_cast<int>(num_particles_ * _goal_distribution[i]);
            total_but_last += particles_per_goal_vector[i];
        }
        else
        {
            // Make the particles sum up to the total number of particles
            particles_per_goal_vector[i] = static_cast<int>(particles_.size() - total_but_last);
        }
    }

    // Create new particle vector by updating the goal position and the weights of the particles.
    double sum_weights = 0.0;
    for (int i = 0; i < particles_per_goal_vector.size(); ++i)
    {
        for (int j = 0; j < particles_per_goal_vector[i]; ++j)
        {
            auto *particle = static_cast<CarState *>(model_->Allocate(-1, _goal_distribution[i]));
            particle->car_pose = decoded_observation.get_car_pose();
            particle->ped_pose = decoded_observation.get_pedestrian_position();
            particle->car_velocity = decoded_observation.get_car_velocity();
            particle->ped_goal = all_goals[i];
            particle->ped_velocity = avg_vel;
            sum_weights += _goal_distribution[i];
            updated_particles.push_back(particle);
        }
    }
    // Normalize weights
    for (auto &particle : updated_particles)
    {
        particle->weight /= sum_weights;
    }

    // ****************************************************************************************************************
    // CLEAN UP
    // ****************************************************************************************************************

    // Store new goal distribution in the class
    _current_distribution = new_distribution;

    // Delete old particles
    for (auto &particle : particles_)
    {
        _car_model->Free(particle);
    }
    particles_.clear();
    particles_ = updated_particles;

    updated_particles.clear();
    current_distribution.clear();
    _last_time = current_time;
}


std::vector<double> CarBelief::ComputeGoalDistribution() const
{
    std::vector<int> counter = {0, 0};
    std::vector<geometry_msgs::Point> goals = _car_model->get_pedestrian_goals();

    for (auto &particle : particles_)
    {
        CarState *s = _car_model->Copy(particle);
        for (int i = 0; i < _num_goals; ++i)
        {
            if (s->ped_goal == goals[i])
            {
                counter[i]++;
            }
        }
    }

    std::vector<double> distribution = {0, 0};
    for (int i = 0; i < _num_goals; ++i)
    {
        distribution[i] = ((double) counter[i]) / num_particles_;
    }

    return distribution;
}

std::vector<std::vector<double>> CarBelief::ComputeFullDistribution() const
{
    // Reserve memory for distribution and counter
    std::vector<std::vector<double>> distribution(_num_goals);
    std::vector<std::vector<int>> counter(_num_goals);
    for (int i = 0; i < _num_goals; ++i)
    {
        distribution[i] = {0, 0};
        counter[i] = {0, 0};
    }

    // Initialize counter to 0
    for (int i = 0; i < _num_goals; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            counter[i][j] = 0;
        }
    }


    std::vector<geometry_msgs::Point> goals = _car_model->get_pedestrian_goals();

    // Count occurrence of goal and type
    for (auto &particle : particles_)
    {
        CarState *state = _car_model->Copy(particle);
        for (int goal_idx = 0; goal_idx < _num_goals; ++goal_idx)
        {
            for (int i = 0; i < 2; ++i)
            {
                if (state->ped_goal == goals[goal_idx] && state->trust == i)
                {
                    counter[goal_idx][i]++;
                }
            }
        }
    }

    // Compute distribution
    for (int goal_idx = 0; goal_idx < _num_goals; ++goal_idx)
    {
        for (int i = 0; i < 2; ++i)
        {
            distribution[goal_idx][i] = (double) counter[goal_idx][i] / num_particles_;
        }
    }

    for (int i = 0; i < _num_goals; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            if (distribution[i][j] < 0.01)
            {
                distribution[i][j] = 0.01;
            }
        }
    }


    return distribution;
}

std::vector<double> CarBelief::ComputeTypeDistribution() const
{
    std::vector<int> counter = {0, 0};
    std::vector<double> distribution = {0, 0};

    // Count types occurrence
    for (auto &particle : particles_)
    {
        CarState *state = _car_model->Copy(particle);
        for (int i = 0; i < 2; ++i)
        {
            if (state->trust == i)
            {
                counter[i]++;
            }
        }
    }

    // Compute distribution
    for (int i = 0; i < 2; ++i)
    {
        distribution[i] = (double) counter[i] / num_particles_;
    }

    return distribution;
}


std::vector<State *> CarBelief::Sample(int num) const
{
    return ParticleBelief::Sample(num);
}

void CarBelief::AddVelocityToFile()
{
    _file.open("/home/albert/velocity", std::ios::app);
    if (_file.is_open())
    {
        _file << current_velocity << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }

    _file.close();
}

void CarBelief::AddPedPositionToFile()
{
    _file.open("/home/albert/ped_pos", std::ios::app);
    if (_file.is_open())
    {
        _file << pedestrian_position.x << " " << pedestrian_position.y << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }

    _file.close();
}

void CarBelief::AddRealGoalToFile()
{
    _file.open("/home/albert/real_goal", std::ios::app);
    if (_file.is_open())
    {
        _file << active_goal.x << " " << active_goal.y << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }

    _file.close();
}

void CarBelief::AddDistributionToFile(std::vector<double> distribution)
{
    _file.open("/home/albert/data", std::ios::app);
    if (_file.is_open())
    {
        unsigned long size = distribution.size();
        for (auto i = 0; i < size; ++i)
        {
            if (i < size - 1)
            {
                _file << distribution[i] << " ";
            }
        }
        // Write the last one to the file.
        _file << distribution[size - 1] << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }
    _file.close();

}

void CarBelief::AddCarPosToFile()
{
    _file.open("/home/albert/car_pos", std::ios::app);
    if (_file.is_open())
    {
        _file << car_pos.x << " " << car_pos.y << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }

    _file.close();
}


void CarBelief::InitializeSubscriber()
{
    active_goal_sub = nhh.subscribe("/active_goal", 100, cb_get_active_goal);
    vel_sub = nhh.subscribe("/odom", 100, cb_get_current_velocity);
    pos_sub = nhh.subscribe("/pedestrian_position", 100, cb_get_pedestrian_position);
    car_pos_sub = nhh.subscribe("/odom", 100, cb_get_car_position);
}

void CarBelief::AddTypeToFile()
{
    _file.open("/home/albert/type", std::ios::app);
    if (_file.is_open())
    {
        _file << _type_distribution[0] << " " << _type_distribution[1] << "\n";
    }
    else
    {
        std::cout << "\033[31mCouldn't write to data text file!\033[0m\n";
    }

    _file.close();
}


} // namespace despot