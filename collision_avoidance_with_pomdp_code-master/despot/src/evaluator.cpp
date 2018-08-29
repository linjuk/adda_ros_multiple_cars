#include <despot/evaluator.h>
#include "ros/ros.h"
#include "car_model/ActionObservation.h"
#include "../examples/cpp_models/car/include/Car.h"
#include "../examples/cpp_models/car/include/pomdp_model/Observation.h"
#include "../examples/cpp_models/car/src/util/FileWriter.hpp"

using namespace std;

namespace despot {

/* =============================================================================
 * EvalLog class
 * =============================================================================*/

time_t EvalLog::start_time = 0;
double EvalLog::curr_inst_start_time = 0;
double EvalLog::curr_inst_target_time = 0;
double EvalLog::curr_inst_budget = 0;
double EvalLog::curr_inst_remaining_budget = 0;
int EvalLog::curr_inst_steps = 0;
int EvalLog::curr_inst_remaining_steps = 0;
double EvalLog::allocated_time = 1.0;
double EvalLog::plan_time_ratio = 1.0;

EvalLog::EvalLog(string log_file) :
    log_file_(log_file) {

    ifstream fin(log_file_.c_str(), ifstream::in);
    if (!fin.good() || fin.peek() == ifstream::traits_type::eof()) {
        time(&start_time);
    } else {
        fin >> start_time;

        int num_instances;
        fin >> num_instances;
        for (int i = 0; i < num_instances; i++) {
            string name;
            int num_runs;
            fin >> name >> num_runs;
            runned_instances.push_back(name);
            num_of_completed_runs.push_back(num_runs);
        }
    }
    fin.close();
}

void EvalLog::Save() {
    ofstream fout(log_file_.c_str(), ofstream::out);
    fout << start_time << endl;
    fout << runned_instances.size() << endl;
    for (int i = 0; i < runned_instances.size(); i++)
        fout << runned_instances[i] << " " << num_of_completed_runs[i] << endl;
    fout.close();
}

void EvalLog::IncNumOfCompletedRuns(string problem) {
    bool seen = false;
    for (int i = 0; i < runned_instances.size(); i++) {
        if (runned_instances[i] == problem) {
            num_of_completed_runs[i]++;
            seen = true;
        }
    }

    if (!seen) {
        runned_instances.push_back(problem);
        num_of_completed_runs.push_back(1);
    }
}

int EvalLog::GetNumCompletedRuns() const {
    int num = 0;
    for (int i = 0; i < num_of_completed_runs.size(); i++)
        num += num_of_completed_runs[i];
    return num;
}

int EvalLog::GetNumRemainingRuns() const {
    return 80 * 30 - GetNumCompletedRuns();
}

int EvalLog::GetNumCompletedRuns(string instance) const {
    for (int i = 0; i < runned_instances.size(); i++) {
        if (runned_instances[i] == instance)
            return num_of_completed_runs[i];
    }
    return 0;
}

int EvalLog::GetNumRemainingRuns(string instance) const {
    return 30 - GetNumCompletedRuns(instance);
}

double EvalLog::GetUsedTimeInSeconds() const {
    time_t curr;
    time(&curr);
    return (double) (curr - start_time);
}

double EvalLog::GetRemainingTimeInSeconds() const {
    return 24 * 3600 - GetUsedTimeInSeconds();
}

// Pre-condition: curr_inst_start_time is initialized
void EvalLog::SetInitialBudget(string instance) {
    curr_inst_budget = 0;
    if (GetNumRemainingRuns() != 0 && GetNumRemainingRuns(instance) != 0) {
        cout << "Num of remaining runs: curr / total = "
            << GetNumRemainingRuns(instance) << " / " << GetNumRemainingRuns()
            << endl;
        curr_inst_budget = (24 * 3600 - (curr_inst_start_time - start_time))
            / GetNumRemainingRuns() * GetNumRemainingRuns(instance);
        if (curr_inst_budget < 0)
            curr_inst_budget = 0;
        if (curr_inst_budget > 18 * 60)
            curr_inst_budget = 18 * 60;
    }
}

double EvalLog::GetRemainingBudget(string instance) const {
    return curr_inst_budget
        - (get_time_second() - EvalLog::curr_inst_start_time);
}

/* =============================================================================
 * Evaluator class
 * =============================================================================*/

Evaluator::Evaluator(Car* model, string belief_type, Solver* solver,
    clock_t start_clockt, ostream* out) :
    model_(model),
    belief_type_(belief_type),
    solver_(solver),
    start_clockt_(start_clockt),
    target_finish_time_(-1),
    out_(out) {
}

Evaluator::~Evaluator() {
}


bool Evaluator::RunStep(int step, int round) {
    if (target_finish_time_ != -1 && get_time_second() > target_finish_time_) {
        if (!Globals::config.silence && out_)
            *out_ << "Exit. (Total time "
                << (get_time_second() - EvalLog::curr_inst_start_time)
                << "s exceeded time limit of "
                << (target_finish_time_ - EvalLog::curr_inst_start_time) << "s)"
                << endl
                << "Total time: Real / CPU = "
                << (get_time_second() - EvalLog::curr_inst_start_time) << " / "
                << (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
                << endl;
        exit(1);
    }

    double step_start_t = get_time_second();

    double start_t = get_time_second();

    // Get the optimal action based on the current belief
    FileWriter::Write("DEBUG", "[Evaluator::RunStep] Searching best action...");
    int action = solver_->Search().action;
    FileWriter::Write("DEBUG", "[Evaluator::RunStep] Done Searching best action!");

    double end_t = get_time_second();
    logd << "[RunStep] Time spent in " << typeid(*solver_).name()
         << "::Search(): " << (end_t - start_t) << endl;




    double reward = 0;
    OBS_TYPE obs = 0;
    start_t = get_time_second();
    bool terminal = ExecuteAction(action, reward, obs);
    end_t = get_time_second();
    logd << "[RunStep] Time spent in ExecuteAction(): " << (end_t - start_t)
         << endl;

    start_t = get_time_second();

    printer.draw_top();
    printer.write("Round " + std::to_string(round) + " Step " + std::to_string(step), printer.GREY, printer.CENTER);
    printer.draw_bottom();
    if (!Globals::config.silence && out_) {
        model_->PrintAction(action, printer);
    }

    if (!Globals::config.silence && out_) {
        model_->PrintObs(*state_, obs, printer);
    }

    const Belief * b = solver_->belief();
    model_->PrintBelief(*b, printer);

    ReportStepReward(printer);
    end_t = get_time_second();

    double step_end_t;
    if (terminal) {
        step_end_t = get_time_second();
        logi << "[RunStep] Time for step: actual / allocated = "
            << (step_end_t - step_start_t) << " / " << EvalLog::allocated_time
            << endl;
        if (!Globals::config.silence && out_)
            *out_ << endl;
        step_++;
        return true;
    }

    *out_<<endl;

    solver_->Update(action, obs);

    step_++;
    return false;
}

double Evaluator::AverageUndiscountedRoundReward() const {
    double sum = 0;
    for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
        double reward = undiscounted_round_rewards_[i];
        sum += reward;
    }
    return undiscounted_round_rewards_.size() > 0 ? (sum / undiscounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrUndiscountedRoundReward() const {
    double sum = 0, sum2 = 0;
    for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
        double reward = undiscounted_round_rewards_[i];
        sum += reward;
        sum2 += reward * reward;
    }
    int n = undiscounted_round_rewards_.size();
    return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}


double Evaluator::AverageDiscountedRoundReward() const {
    double sum = 0;
    for (int i = 0; i < discounted_round_rewards_.size(); i++) {
        double reward = discounted_round_rewards_[i];
        sum += reward;
    }
    return discounted_round_rewards_.size() > 0 ? (sum / discounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrDiscountedRoundReward() const {
    double sum = 0, sum2 = 0;
    for (int i = 0; i < discounted_round_rewards_.size(); i++) {
        double reward = discounted_round_rewards_[i];
        sum += reward;
        sum2 += reward * reward;
    }
    int n = discounted_round_rewards_.size();
    return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

void Evaluator::ReportStepReward(Printer printer) {
    if (!Globals::config.silence && out_) {
//        printer.write("Rewards:", printer.PURPLE);
//        printer.newline();
//        printer.write("                Current Rewards: ");
//        printer.write(std::to_string(reward_));
//        printer.newline();
//        printer.write("      discounted / undiscounted: ");
//        printer.write(std::to_string(total_discounted_reward_));
//        printer.write(" / " + std::to_string(total_undiscounted_reward_));
//
//        printer.draw_bottom();
        *out_ << "- Reward = " << reward_ << endl
              << "- Current rewards:" << endl
              << "  discounted / undiscounted = " << total_discounted_reward_
              << " / " << total_undiscounted_reward_ << endl;
    }
}

/* =============================================================================
 * POMDPEvaluator class
 * =============================================================================*/

POMDPEvaluator::POMDPEvaluator(DSPOMDP* model, string belief_type,
    Solver* solver, clock_t start_clockt, ostream* out,
    double target_finish_time, int num_steps) :
    Evaluator(static_cast<Car*>(model), belief_type, solver, start_clockt, out),
    random_((unsigned) 0)
{
    target_finish_time_ = target_finish_time;

    if (target_finish_time_ != -1) {
        EvalLog::allocated_time = (target_finish_time_ - get_time_second())
            / num_steps;
        Globals::config.time_per_move = EvalLog::allocated_time;
        EvalLog::curr_inst_remaining_steps = num_steps;
    }

    ros::NodeHandle nhh("");
    nh = nhh;
    client = nh.serviceClient<car_model::ActionObservation>("/info_exchanger", true);

    if (!client.exists()) {
        default_out << "\033[1;31mCouldn't connect to Service !\033[0m\n";
        exit(1);
    } else {
        default_out << "\n\033[1;32mConnected to Service !\033[0m\n";
    }
}

POMDPEvaluator::~POMDPEvaluator() {
}

int POMDPEvaluator::Handshake(string instance) {
    return -1; // Not to be used
}

void POMDPEvaluator::InitRound() {
    step_ = 0;

    double start_t, end_t;

    // Create Initial State
    state_ = model_->CreateStartState("DEFAULT");
    logd << "[POMDPEvaluator::InitRound] Created start state." << endl;
    if (!Globals::config.silence && out_) {
        *out_ << "Initial state: " << endl;
        model_->PrintState(*state_, *out_);
        *out_ << endl;
    }

    // Create Initial Belief
    start_t = get_time_second();
    delete solver_->belief();
    end_t = get_time_second();
    logd << "[POMDPEvaluator::InitRound] Deleted old belief in "
        << (end_t - start_t) << "s" << endl;

    start_t = get_time_second();
    Belief* belief = model_->InitialBelief(state_, "UNIFORM");
    end_t = get_time_second();
    logd << "[POMDPEvaluator::InitRound] Created intial belief "
        << typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;

    // Set the Belief for the solver
    solver_->belief(belief);

    total_discounted_reward_ = 0;
    total_undiscounted_reward_ = 0;
}

double POMDPEvaluator::EndRound() {
    if (!Globals::config.silence && out_) {
        *out_ << "Total discounted reward = " << total_discounted_reward_ << endl
            << "Total undiscounted reward = " << total_undiscounted_reward_ << endl;
    }

    discounted_round_rewards_.push_back(total_discounted_reward_);
    undiscounted_round_rewards_.push_back(total_undiscounted_reward_);

    return total_undiscounted_reward_;
}

bool POMDPEvaluator::ExecuteAction(int action, double& reward, OBS_TYPE& obs)
{
    ros::Time current_time = ros::Time::now();
    auto passed = current_time.toSec() - _last_time.toSec();

    std::cout << "Passed time before waiting: " << passed << std::endl;
    // If DESPOT was faster than 1 second (one time step), wait a bit
    while (passed < 1.0)
    {
        current_time = ros::Time::now();
        passed = current_time.toSec() - _last_time.toSec();
    }

    // Create an service instance and fill the 'action' variable
    car_model::ActionObservation srv;
    srv.request.action = action;
    bool terminal = false;

    if (client.call(srv))
    {
        // Get the observation (response)
        geometry_msgs::Pose car_pose = srv.response.car_pose;
        geometry_msgs::Pose ped_pose = srv.response.pedestrian_position;
        double car_v = srv.response.car_v;

        // Save observation
        Observation observation = Observation(car_pose, ped_pose, car_v);
        obs = Observation::EncodeObservation(observation);

        // Reward calculations
        if (action == model_->ACCELERATE)
            reward += model_->reward_acceleration_;
        else if (action == model_->DECELERATE)
            reward += model_->reward_deceleration_;

        // Calculate reward for high speeds.
        reward += model_->HighSpeedReward(car_v);

        // Calculate reward for possible pedestrian collision.
        if (Car::in_range(car_pose.position, ped_pose.position, model_->DISTANCE_PED_TO_CAR)) {
            reward += model_->reward_ped_in_range_;
        }

        // Calculate reward for car getting closer to the goal.
        if (Car::in_range(car_pose.position, model_->get_car_goal(), model_->DISTANCE_CAR_TO_GOAL)) {
            reward += model_->reward_car_near_goal_;
        }

        // Dirty way of setting the car's velocity to 0
        if (Car::in_range(car_pose.position, model_->get_car_goal(), model_->DESPOT_TERMINAL_RADIUS)
            || Car::in_range(car_pose.position, ped_pose.position, model_->DESPOT_TERMINAL_RADIUS)) {
            terminal = true;
            srv.request.action = 3;
            client.call(srv);
        }

        /* Get rid of the first point in the path as it is done and re-assign the original path. */
//        Car::original_path.erase(Car::original_path.begin());
//        Car::path = Car::original_path;
//        default_out << "Path size: " << Car::path.size() << std::endl;
    }
    else
    {
        // Something went wrong with the call, exit
        default_out << "\033[1;31mCouldn't get observation!\033[0m\n";
        srv.request.action = 3;
        client.call(srv);
        exit(1);
    }

    // Summarize rewards
    reward_ = reward;
    total_discounted_reward_ += Globals::Discount(step_) * reward;
    total_undiscounted_reward_ += reward;

    _last_time = current_time;

    return terminal;
}

double POMDPEvaluator::End() {
    return 0; // Not to be used
}

void POMDPEvaluator::UpdateTimePerMove(double step_time) {
    if (target_finish_time_ != -1) {
        if (step_time < 0.99 * EvalLog::allocated_time) {
            if (EvalLog::plan_time_ratio < 1.0)
                EvalLog::plan_time_ratio += 0.01;
            if (EvalLog::plan_time_ratio > 1.0)
                EvalLog::plan_time_ratio = 1.0;
        } else if (step_time > EvalLog::allocated_time) {
            double delta = (step_time - EvalLog::allocated_time)
                / (EvalLog::allocated_time + 1E-6);
            if (delta < 0.02)
                delta = 0.02; // Minimum reduction per step
            if (delta > 0.05)
                delta = 0.05; // Maximum reduction per step
            EvalLog::plan_time_ratio -= delta;
            // if (EvalLog::plan_time_ratio < 0)
            // EvalLog::plan_time_ratio = 0;
        }

        EvalLog::curr_inst_remaining_budget = target_finish_time_
            - get_time_second();
        EvalLog::curr_inst_remaining_steps--;

        if (EvalLog::curr_inst_remaining_steps <= 0) {
            EvalLog::allocated_time = 0;
        } else {
            EvalLog::allocated_time =
                (EvalLog::curr_inst_remaining_budget - 2.0)
                    / EvalLog::curr_inst_remaining_steps;

            if (EvalLog::allocated_time > 5.0)
                EvalLog::allocated_time = 5.0;
        }

        Globals::config.time_per_move = EvalLog::plan_time_ratio
            * EvalLog::allocated_time;
    }
}
} // namespace despot
