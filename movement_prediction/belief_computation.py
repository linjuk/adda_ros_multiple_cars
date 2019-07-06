# System Imports
import tf
import time
import rospy
import numpy as np
import abc
import pickle
import math
from collections import defaultdict

# ROS Messages
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

# Custom Messages
from promp_robot_control_msgs.msg import PredictedTrajectories

# Custom Imports
from history import *
from scipy.stats import multivariate_normal
from numpy.linalg import multi_dot


import sys
sys.path.append('../')
import learn_trajectories as promps

# Plots (for performance analysis)
import matplotlib.pyplot as plt


class BeliefComputationMethod(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, belief_tracker, frequency, use_ros, belief_tracker_only):
        self._belief_tracker = belief_tracker
        self._frequency = frequency
        self.use_ros = use_ros
        self._belief_tracker_only = belief_tracker_only
        self.initialize_parameters()
    

    @abc.abstractmethod
    def _compute_belief_from_pose_and_time(self):
        pass


    def initialize_parameters(self):
        self.update_factor_transition_probabilities_online = True
        self.update_factor_transition_probabilities = 0.1
        self.goals_reached_history =[]

        self.current_init_belief=[]

        self._estimated_positions = []
        self._current_human_pos = []
        self._current_belief = []
        self._direction = []
        self._goals = []
        self._velocities = []
        self._last_belief_over_history = []

        self._num_sampled_trajectories = 20

        self._reached_goals = []
        self._num_goals = 0
        self._human_std_dev = 0.01
        self._obs_variance = 0.0005 #0.002 #0.0005

        self._radius_around_goal = [0.1 for _ in range(self._num_goals)] # should be different for every goal

        self._last_time = time.time()
        self._history = History()

        self._max_belief_history = int(self._frequency * 0.6) #1.0
        self._max_vel_history = int(self._frequency * 0.3)# *1.0


    def update_belief_tracker_parameters(self,filename, belief_tracker_goal_transition_history):

        with open(filename, 'rb') as handle:
            data = pickle.load(handle)

            transition_probabilities_history_vector = data["transition_probabilities_history_vector"]
            _goal_change_prob_all = data["transition_probabilities"]
            resting_duration = data["resting_duration"]


        idx_history_length = np.where(transition_probabilities_history_vector == belief_tracker_goal_transition_history)[0]

        if len(idx_history_length) == 1:
            self.transition_probabilities_history_vector = transition_probabilities_history_vector
            self._goal_change_prob_all = _goal_change_prob_all
            self.idx_history_change_probs = idx_history_length[0]
            self._goal_change_prob = self._goal_change_prob_all[idx_history_length[0]]
            self._goal_change_prob_length = belief_tracker_goal_transition_history

            self._resting_duration_at_goals = np.array([0.,0.0,0.,0.])#resting_duration
            #print(resting_duration)
            self.curr_stay_duration_goals = np.copy(self._resting_duration_at_goals)

        else:
            print "requested history length not or multiple times in specified history vector not updating"


    def normalize(self, v):
        """
        Normalizes a vector.
        Parameters
        ----------
        v : np.ndarray
            Vector to normalize

        Returns
        -------
        np.ndarray :
            Normalized vector.

        """
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm


    def _at_goal(self, position, goal, threshold):
        # type: (np.ndarray, np.ndarray, float) -> bool
        return np.linalg.norm(goal - position) <= threshold
    

    def _is_human_in_range_of_goal(self, current_position):
        # type: (np.ndarray) -> bool
        for g in range(self._num_goals):
            if np.linalg.norm(self._goals[g] - current_position) <= self._radius_around_goal[g]:
                return [True, g]

        return [False, -1]


    def _compute_velocity(self, last_pos, current_pos, dt):
        # type: (np.ndarray, np.ndarray, float) -> (float, float)
        """
        Computes the current velocity, as well as the average velocity over the history.
        :param last_pos:
        :param current_pos:
        :param dt: Time step
        :return: Current velocity and average velocity
        """
        diff = current_pos - last_pos
        raw_vel = np.sqrt(np.dot(diff, diff)) / dt
        # do we need this?
        # if raw_vel > self._max_velocity:
        #     raw_vel = self._max_velocity - np.random.normal(0, 0.01)
        ####### avg_vel = (np.sum(self._history.velocities) + raw_vel) / (self._history.size() + 1)
        self._velocities.append(raw_vel)
        avg_vel = float(np.mean(self._velocities[-self._max_vel_history:]))

        return float(raw_vel), float(avg_vel)


    def _compute_direction(self, current_observed_human_point):
        """
        Computes the direction the human is currently heading towards.

        Parameters
        ==========
        current_observed_human_point : np.ndarray
            Current position of the human wrist.

        Returns
        =======
        np.ndarray :
            Direction
        """

        if self._history.size() < self._max_belief_history:
            start_idx_history_window = 1
        else:
            start_idx_history_window = self._history.size() - self._max_belief_history

        return current_observed_human_point - self._history.observations[start_idx_history_window - 1]


    def _init_belief(self, current_goal=-1):
        """
        Initializes the belief uniformly between the goals 
        (except the current one if current_goal is not -1).
        """
        belief = []

        if self._num_goals < 2:
            return belief

        if current_goal == -1:
            uniform_belief = 1.0 / self._num_goals
        else:
            uniform_belief = 1.0 / (self._num_goals - 1)

        for i in range(self._num_goals):
            if i == current_goal:
                belief.append(0.00001)
            else:
                belief.append(uniform_belief)

        self._last_belief_over_history = np.copy(belief)
        self._current_belief = belief
        return belief


    def update_belief_close_to_goal(self, current_position, current_belief):
        [is_at_goal, g] = self._is_human_in_range_of_goal(current_position)

        #print " at "+str(g)
        likelihoods = np.zeros(self._num_goals)+0.001
        likelihoods[g] = 1.0
        new_belief = []
        normalization_factor = 0.0

        for goal_idx in range(self._num_goals):
            normalization_factor += likelihoods[goal_idx] * current_belief[goal_idx]

        for goal_idx in range(self._num_goals):
            if normalization_factor==0.0:
                print("norm : " + str(normalization_factor))
                print("likelihoods", likelihoods)
                print("current_belief", current_belief)
            prob = (likelihoods[goal_idx] * current_belief[goal_idx]) / normalization_factor
            new_belief.append(prob)

        return new_belief


class DirectionBeliefComputation(BeliefComputationMethod):
    def __init__(self, belief_tracker, frequency, use_ros, belief_tracker_only):
        BeliefComputationMethod.__init__(self, belief_tracker, frequency, use_ros, belief_tracker_only)
        self._personalized_vel = 0.6
        self._resting_duration_at_goals = []

        self.threshold_goal_reached = 0.15  # TODO this should be consistent with data eval parameters instead of hardcoded

        self._current_velocity = 0.0

        self._max_velocity = 1.0
        self._belief_threshold = 0.8
        self._belief_threshold_min = 0.3

        #self._t_predict = int(self._frequency * 2.0)
        self._t_predict_dt = 0.3 #(1.0 / self._frequency)*6
        #print("dt predict " + str(self._t_predict_dt))
        self._t_predict = int(2.0 / self._t_predict_dt) #2.5 vorher
        #print("T predict " + str(self._t_predict))
        #print("t_pred:"+str(self._t_predict))


    def transition_human(self, position, velocity, goal, dt):
        # type: (np.ndarray, float, np.ndarray, float) -> np.ndarray
        """
        The human model to compute the next position given the current position,
        the velocity and the goal. Returns a biased position vector.
        :param position: Current position of the human
        :param velocity: Current velocity of the human.
        :param goal: Goal position of the human.
        :param dt: Time difference.
        :return: New position.
        """

        std_noise=np.copy(self._human_std_dev)
        if velocity <= 0.2:
            std_noise = std_noise*velocity

        noise = np.random.normal(loc=0.0, scale=std_noise, size=3)
        change_in_position = (velocity * dt * self.normalize(goal - position))
        dist_to_goal=np.linalg.norm(goal-position)
        dist_change = np.linalg.norm(change_in_position)
        if dist_change>dist_to_goal:
            change_in_position = goal - position

        change_in_position+= noise
        return position + change_in_position


    def compute_observation_likelihood(self, current_observation, last_observation, goal, velocity, dt):
        # type: (np.ndarray, np.ndarray, np.ndarray, float, float) -> (float, np.ndarray)
        """Computes the likelihood for the current observation.
        :param current_observation: Current observation
        :param last_observation: Last observation
        :param goal: Goal that the human is heading towards to
        :param velocity: Velocity at which the human moves
        :param dt: Time step
        :return: Likelihood
        """
        calculated_position = self.transition_human(last_observation, velocity, goal, dt)
        likelihood = multivariate_normal.pdf(x=current_observation, mean=calculated_position,
                                             cov=(self._obs_variance * np.identity(3)))
        return likelihood, calculated_position


    def update_belief_once(self, current_observation, last_observation, avg_vel, dt, current_belief):
        # type: (np.ndarray, np.ndarray, float, float, list) -> (list, list)
        """Updates the belief one time.
        :param current_observation: Current observation
        :param last_observation: Last observation
        :param avg_vel: Average velocity
        :param dt: Time step
        :param current_belief: Current Belief
        :return: Updated belief and estimated positions
        """

        new_belief = []
        likelihoods = []
        estimated_positions = []
        normalization_factor = 0.0

        # Compute the likelihoods
        for goal_idx in range(self._num_goals):
            obs_likelihood, calculated_position = self.compute_observation_likelihood(current_observation,
                                                                                      last_observation,
                                                                                      self._goals[goal_idx],
                                                                                      avg_vel, dt)
            estimated_positions.append(calculated_position)
            obs_likelihood += 1
            likelihoods.append(obs_likelihood)
            normalization_factor += obs_likelihood * current_belief[goal_idx]


        #for i in range(self.importance_of_prior_in_belief_update):
        #normalization_factor = 0.0
        #tmp_belief = []
        # Compute new belief
        for goal_idx in range(self._num_goals):
            prob = (likelihoods[goal_idx] * current_belief[goal_idx])/normalization_factor

            new_belief.append(prob)

        #tmp_belief = np.array(tmp_belief) / normalization_factor

        #new_belief = tmp_belief
        return [new_belief, estimated_positions]


    def _update_belief_over_history(self):
        # type: () -> list
        """
        Updates the belief over the history (last n entries).
        History has to have at least 2 entries!
        The current observation will not be included here!
        :return: The new belief
        """

        if len(self.current_init_belief)==0:
            #print"init normal:"
            belief = self._init_belief()
        else:
            #print"init transitions:"
            #print self.current_init_belief
            belief = np.copy(self.current_init_belief)
            self._last_belief_over_history = np.copy(belief)

        #print "after"
        if self._history.size() < self._max_belief_history:
            start_idx_history_window = 1
        else:
            start_idx_history_window = self._history.size() - self._max_belief_history

        for t in range(start_idx_history_window, self._history.size()):
            dt = self._history.dts[t]
            current_obs = self._history.observations[t]
            last_obs = self._history.observations[t - 1]

            vel, avg_vel = self._compute_velocity(last_obs, current_obs, dt)

            # TODO: Belief will be computed multiple times here too!
            belief, _ = self.update_belief_once(current_obs, last_obs, avg_vel, dt, belief)

        return belief


    def compute_transition_probs_for_goal_from_history(self, history,selected_goal_idx):
        # compute probabilities for all goals
        available_probs = []
        available_goals = []
        sum_p = 0.0
        for g in range(self._num_goals):
            try:  # TODO remove this since not using dictionary anymore

                # if first_goal_idx == 0:
                #     selected_goal_idx_selection = 0

                if self._goal_change_prob_length > 1:  # will break for more then one goal
                    # print "in if history >1"

                    if not (selected_goal_idx == history[-1]):

                        tmp = np.copy(history[-self._goal_change_prob_length + 1:len(history)])
                    else:
                        if len(history) >= self._goal_change_prob_length + 1:

                            tmp = np.copy(history[
                                          -self._goal_change_prob_length + 1 - 1:len(history) - 1])
                        else:

                            tmp = np.copy(history[0:len(history) - 1])

                    tmp = tmp.tolist()
                    # print"tmp1:"
                    # print tmp
                    #
                    # print "skipping equal belief last history"
                    # else:
                    # print"ok"

                    tmp.append(selected_goal_idx)

                    # print"tmp2:"
                    # print tmp
                    selected_goal_idx_selection = tmp

                    selected_goal_idx_selection.append(g)
                else:
                    # print "in else history <1, selcted goal idx:"
                    # print selected_goal_idx
                    selected_goal_idx_selection = [selected_goal_idx, g]
                # print "reached goals 2:"
                # print self._reached_goals
                # print "sel idx:"
                # print selected_goal_idx_selection
                if len(selected_goal_idx_selection) == self._goal_change_prob_length + 1:
                    prob = self._goal_change_prob[tuple(selected_goal_idx_selection)]
                else:
                    # print "reached:"
                    # print self._reached_goals
                    print "history not yet long enough"
                    idx_history_length = \
                    np.where(self.transition_probabilities_history_vector == len(selected_goal_idx_selection) - 1)[0]

                    if len(idx_history_length) == 1:
                        prob = self._goal_change_prob_all[idx_history_length[0]][tuple(selected_goal_idx_selection)]

                    else:
                        # print "no data known for history size "+ str(len(selected_goal_idx)-1) +" updating with uninformed priors"
                        prob = 1. / (len(self._goals) - 1)

                # print prob
                if prob < 0.0:
                    # print "reached:"
                    # print self._reached_goals
                    # print "_change length:" + str(self._goal_change_prob_length)
                    history_l = np.copy(np.min([self._goal_change_prob_length, len(history)]))
                    history_l -= 1
                    # print "history_l"
                    # print history_l

                    # print "prob"
                    # print prob
                    # print "goal selected idx:"
                    # print selected_goal_idx_selection

                    while prob < 0.0 and history_l >= 1:
                        tmp_idx = tuple(selected_goal_idx_selection[
                                        np.min([self._goal_change_prob_length, len(history)]) - history_l: len(
                                            selected_goal_idx_selection)])

                        prob = self._goal_change_prob_all[history_l - 1][tmp_idx]
                        history_l -= 1

                    if prob < 0.0:
                        print "situation: "
                        print selected_goal_idx_selection
                        print "unknown -> updating with uninformed priors"
                        prob = 1. / (len(self._goals) - 1)
                    else:
                        print "situation: "
                        print selected_goal_idx_selection
                        print "unknown -> using:"
                        print selected_goal_idx_selection[
                              self._goal_change_prob_length - (history_l + 1):len(selected_goal_idx_selection)]
                # else:
                # print "(ok) "
                # print self._reached_goals
                sum_p += prob
                available_goals.append(g)
                available_probs.append(prob)
            except KeyError:
                continue

        probs = [v / sum_p for v in available_probs]

        return probs, available_goals


    def update_transition_probabilities(self):
        goals = self._reached_goals
        print "reached goals"
        print goals
        print "all 0:"
        #print self._goal_change_prob_all[0]

        for i in range(len(self._goal_change_prob_all)):
            #print "dims"
            #print self._goal_change_prob_all[i].ndim
            if self._goal_change_prob_all[i].ndim <= len(self._reached_goals):
                indices = self._reached_goals[
                          len(self._reached_goals) - self._goal_change_prob_all[i].ndim:len(self._reached_goals)]
                indices_small = self._reached_goals[
                          len(self._reached_goals) - self._goal_change_prob_all[i].ndim:len(self._reached_goals)-1]
                first_time = False
                for g in range(self._num_goals):
                    idx_tmp = indices_small[:]
                    idx_tmp.append(g)
                    if self._goal_change_prob_all[i][tuple(idx_tmp)]<0.0:
                        first_time=True
                        self._goal_change_prob_all[i][tuple(idx_tmp)] = 0.0
                        print "updating -1 probabilitiy"

                    self._goal_change_prob_all[i][tuple(idx_tmp)]*=  (1.-self.update_factor_transition_probabilities)

                # if self._goal_change_prob_all[i].ndim == 2:
                #     print "idx:"
                #     print indices
                #
                #     self._goal_change_prob_all[i][: , self._reached_goals[-1]] *=  (1.-self.update_factor_transition_probabilities)
                # elif self._goal_change_prob_all[i].ndim == 3:
                #     self._goal_change_prob_all[i][:,:, self._reached_goals[-1]] *= (1. - self.update_factor_transition_probabilities)
                # elif self._goal_change_prob_all[i].ndim == 4:
                #     self._goal_change_prob_all[i][:,:,:, self._reached_goals[-1]] *= (1. - self.update_factor_transition_probabilities)
                # elif self._goal_change_prob_all[i].ndim == 5:
                #     self._goal_change_prob_all[i][:,:,:,:, self._reached_goals[-1]] *= (1. - self.update_factor_transition_probabilities)
                #print "dims"
                #self._goal_change_prob_all[i].ndim

                # print "idx:"
                # print indices
                if i ==3:
                    print "idx:"
                    print indices
                    print "idx_small:"
                    print indices_small
                if first_time:
                    self._goal_change_prob_all[i][tuple(indices)] = 1.0
                else:
                    self._goal_change_prob_all[i][tuple(indices)] += 1.0*self.update_factor_transition_probabilities

        print (self._goal_change_prob_all[0].shape)

        #for i in range(self._num_goals):

        self._goal_change_prob = self._goal_change_prob_all[self.idx_history_change_probs]
        #print self._goal_change_prob
        # print"0|1"
        # print self._goal_change_prob[1,0]
        # print"0|3"
        # print self._goal_change_prob[ 3,0]

        print "p(3|1,0,2,0):  " + str( self._goal_change_prob_all[3][1,0,2,0,3])

        print "p(0|0,2,0,3):  " + str(self._goal_change_prob_all[3][0, 2, 0, 3, 0])
        print "p(1|0,2,0,3):  " + str(self._goal_change_prob_all[3][0,2,0,3,1])
        print "p(2|0,2,0,3):  " + str(self._goal_change_prob_all[3][0, 2, 0, 3, 2])

        print "p(0|0,1,0,3):  " + str(self._goal_change_prob_all[3][0, 1, 0, 3, 0])
        print "p(1|0,1,0,3):  " + str(self._goal_change_prob_all[3][0,1,0,3, 1])
        print "p(2|0,1,0,3):  " + str(self._goal_change_prob_all[3][0,1, 0, 3, 2])

        print "p(2|0,3):  " + str(self._goal_change_prob_all[1][0, 3, 2])
        print "p(1|0,3):  " + str(self._goal_change_prob_all[1][0, 3, 1])
        print "p(0|0,3):  " + str(self._goal_change_prob_all[1][0, 3, 0])

        print "p(2|1):  " + str(self._goal_change_prob_all[1][1,2])
        print "p(0|1):  " + str(self._goal_change_prob_all[1][1,0])
        print "p(3|1):  " + str(self._goal_change_prob_all[1][1,3])
        #print "p(0|1):  " + str(self._goal_change_prob_all[3][1, 0])
        #print "p(0|1):  " + str(self._goal_change_prob_all[3][1, 0])
        #print "p(0|1):  " + str(self._goal_change_prob_all[3][1, 0])
        #print "p(0|1):  " + str(self._goal_change_prob_all[3][1, 0])


    def _compute_belief_from_pose_and_time(self, current_human_pos, current_time):
        self._current_human_pos = current_human_pos
        # Update the Goals by calling the GoalTracker if goal tracker is active.
        if not self._belief_tracker_only:
            self._belief_tracker._update_goals()

        dt = current_time - self._last_time

        # Store history over reached goals for better transition probability predictions

        for g_idx in range(len(self._goals )):
            goal = np.asarray(self._goals[g_idx])

            if self._at_goal(current_time, current_human_pos, goal, self.threshold_goal_reached):
                if len(self._reached_goals)>0:

                    if not g_idx==self._reached_goals[-1]:
                        self._reached_goals.append(g_idx)
                        probs,_ = self.compute_transition_probs_for_goal_from_history(np.copy(self._reached_goals),g_idx)

                        #self._current_belief = list(np.array(probs)+0.001)

                        #self._history = History()
                        print "recomputing prior"
                        print self._current_belief
                        self.current_init_belief = list(np.copy(probs)+0.001)
                        if self.update_factor_transition_probabilities_online:
                            self.update_transition_probabilities()

                else:

                    self._reached_goals.append(g_idx)
                    probs,_ = self.compute_transition_probs_for_goal_from_history(list(np.copy(self._reached_goals)), np.copy(g_idx))
                    #self._current_belief = list(np.array(probs)+0.001)
                    #self._history = History()
                    print "recomputing prior"
                    print self._current_belief
                    self.current_init_belief = list(np.copy(probs)+0.001)
                    if self.update_factor_transition_probabilities_online:
                        self.update_transition_probabilities()

        # print"reached goals:"
        # print self._reached_goals


        # This is the first observation, just fill up the history
        if self._history.size() < 1:

            # if len(self.current_init_belief) == 0:
            #     print"init normal:"
            #     self._current_belief = self._init_belief()
            # else:
            #     print"init transitions:"
            #     print self.current_init_belief
            #     self._current_belief = self.current_init_belief

            last_obs = current_human_pos
            vel, avg_vel = self._compute_velocity(last_obs, current_human_pos, dt)
            self._current_belief, self._estimated_positions = self.update_belief_once(current_human_pos, last_obs,
                                                                                      avg_vel, dt,
                                                                                      self._current_belief)

            self._last_time = current_time
            self._history.add_entry(current_time, current_human_pos, vel, dt)
            self._direction = self._compute_direction(current_human_pos)

            belief_msg = Float64MultiArray()
            belief_msg.data = self._current_belief
            if self.use_ros:
                self._belief_tracker._pub_belief.publish(belief_msg)

            return [],[],avg_vel

        elif self._history.size() == 1:
            last_obs = self._history.last_entry()[1]
            (vel, avg_vel) = self._compute_velocity(last_obs, current_human_pos, dt)
            self._current_belief, self._estimated_positions = self.update_belief_once(current_human_pos, last_obs,
                                                                                      avg_vel, dt,
                                                                                      self._current_belief)
            self._last_time = current_time
            self._history.add_entry(current_time, current_human_pos, vel, dt)
            self._direction = self._compute_direction(current_human_pos)

            belief_msg = Float64MultiArray()
            belief_msg.data = self._current_belief
            if self.use_ros:
                self._belief_tracker._pub_belief.publish(belief_msg)
                
            return [],[],avg_vel

        # Update the belief with the current observation
        last_obs = self._history.last_entry()[1]
        vel, avg_vel = self._compute_velocity(last_obs, current_human_pos, dt)

        # if avg_vel>=0.21:
        #     print "set velocity to 0.6"
        #     avg_vel=0.6
        # else:
        #     print "not"

        self._current_velocity = np.copy(avg_vel)

        #print(avg_vel)

        [is_at_goal, goal_index] = self._is_human_in_range_of_goal(np.asarray(current_human_pos))
        if not is_at_goal:
            self.curr_stay_duration_goals = np.copy(self._resting_duration_at_goals)
            # Only update the belief, if human is moving
            if avg_vel > 0.2:
                # First, compute the belief over the history (without the current observation)
                belief_over_history = self._update_belief_over_history()

                new_belief, self._estimated_positions = self.update_belief_once(current_human_pos, last_obs, avg_vel, dt,
                                                                                belief_over_history)
                sampled_trajectories = self._generate_trajectories(vel, self._t_predict_dt, self._num_sampled_trajectories, self._t_predict,
                                                                   new_belief)
                self._current_belief = np.copy(new_belief)
                self._last_time = current_time
                self._direction = self._compute_direction(current_human_pos)
                self._last_belief_over_history = belief_over_history

                belief_msg = Float64MultiArray()
                belief_msg.data = self._current_belief
                if self.use_ros:
                    self._belief_tracker._pub_belief.publish(belief_msg)
            else:
                # TODO: Set probability to 1 ,if near a goal?
                # Generate trajectories for prediction
                _, self._estimated_positions = self.update_belief_once(current_human_pos, last_obs, avg_vel, dt,
                                                                       self._current_belief)
                sampled_trajectories = self._generate_trajectories(vel, self._t_predict_dt, self._num_sampled_trajectories, self._t_predict,
                                                                   self._current_belief)
                self._last_time = current_time
                self._direction = self._compute_direction(current_human_pos)
                belief_msg = Float64MultiArray()
                belief_msg.data = self._current_belief
                if self.use_ros:
                    self._belief_tracker._pub_belief.publish(belief_msg)
        # At a goal
        else:
            if avg_vel > 0.2:
                #
                # if self.leaving_goal_first_time:
                #     self.leaving_goal_first_time = False
                #
                #     print "recompute priors"

                # TODO: Set probability to 1 ,if near a goal?
                # Generate trajectories for prediction
                _, self._estimated_positions = self.update_belief_once(current_human_pos, last_obs, avg_vel, dt,
                                                                       self._current_belief)
                sampled_trajectories = self._generate_trajectories(self._personalized_vel, self._t_predict_dt, self._num_sampled_trajectories, self._t_predict,
                                                                   self._current_belief)
                self._last_time = current_time
                self._direction = self._compute_direction(current_human_pos)
                belief_msg = Float64MultiArray()
                belief_msg.data = self._current_belief
                if self.use_ros:
                    self._belief_tracker._pub_belief.publish(belief_msg)
            else:
                # self.leaving_goal_first_time _update_belief_over_history= True
                self.curr_stay_duration_goals[goal_index] =  np.max([0.,self.curr_stay_duration_goals[goal_index] - 1./self._frequency])
                # First, compute the belief over the history (without the current observation)
                belief_over_history = self._update_belief_over_history()

                # TODO: Maybe remove here
                self._estimated_positions = [current_human_pos for _ in range(self._num_goals)]

                new_belief = self.update_belief_close_to_goal(current_human_pos, self._current_belief)
                sampled_trajectories = self._generate_trajectories(self._personalized_vel, self._t_predict_dt, self._num_sampled_trajectories,
                                                                   self._t_predict,
                                                                   new_belief)
                self._current_belief = np.copy(new_belief)
                self._last_time = current_time
                self._direction = self._compute_direction(current_human_pos)
                self._last_belief_over_history = belief_over_history

                belief_msg = Float64MultiArray()
                belief_msg.data = self._current_belief
                if self.use_ros:
                    self._belief_tracker._pub_belief.publish(belief_msg)

        # Add the current observation to the history
        self._history.add_entry(current_time, current_human_pos, vel, dt)

        trajectories = PredictedTrajectories()
        for i in range(len(sampled_trajectories)):
            path = Path()
            for j in range(len(sampled_trajectories[i])):
                p = sampled_trajectories[i][j]
                pose = PoseStamped()
                pose.header.frame_id = "darias"
                pose.header.stamp = rospy.Time(0)
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = p[2]
                path.poses.append(pose)
            trajectories.trajectories.append(path)
        if self.use_ros:
            self._belief_tracker._pub_sampled_trajectories.publish(trajectories)
            self._belief_tracker._publish_visualized_trajectories(sampled_trajectories)

            # Publish current direction as a Marker
            self._belief_tracker._publish_current_direction_and_goal(self._current_belief)

            ma = MarkerArray()
            for i in range(self._num_goals):
                m = Marker()
                m.header.frame_id = "darias"
                m.header.stamp = rospy.Time.now()
                m.ns = "my_namespace"
                m.id = i + 1000
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.pose.position.x = self._goals[i][0]
                m.pose.position.y = self._goals[i][1]
                m.pose.position.z = self._goals[i][2] + 0.1
                m.text = "{0:.4f}".format(self._current_belief[i])
                m.pose.orientation.w = 1.0
                m.color.a = 1.0
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 1.0
                m.scale.x = 0.1
                m.scale.y = 0.1
                m.scale.z = 0.1
                ma.markers.append(m)
            self._belief_tracker._pub_belief_vis.publish(ma)

        return sampled_trajectories, is_at_goal, avg_vel


    def _sample_position(self, positions, current_belief):
        """
        Samples a positions from the given position vector based on the current belief.

        Parameters
        ----------
        positions : List
            List of positions.

        Returns
        -------
        List :
            np.ndarray : New position based on the belief.<br>
            np.ndarray : Sampled goal
        """
        new_belief = np.copy(current_belief)

        # Threshold Belief and re-normalize
        # If we are very sure of one goal we do not care about the others
        for i in range(self._num_goals):
            if current_belief[i] < self._belief_threshold:
                new_belief[i] = 0.0

        # print "probs belief before:"
        # print new_belief

        # if we are very unsure about one goal we do not use it
        if np.max(new_belief) == 0.0:
            new_belief = np.copy(current_belief)
            for i in range(self._num_goals):
                if current_belief[i] < self._belief_threshold_min:
                    new_belief[i] = 0.0
            print "using old belief above min threshold"

        # this should never happen I think unless we have super many goals
        if np.max(new_belief) == 0.0:
            new_belief = np.copy(current_belief)
            print "using old belief, should not happen"

        # print "probs belief:"
        # print new_belief
        new_belief = new_belief / np.sum(new_belief)


        idx = np.random.choice(a=np.arange(len(positions)), p=new_belief)
        return np.asarray(positions[idx]), self._goals[idx]

    
    def _generate_trajectories(self, velocity, dt, num_trajectories, prediction_time, current_belief):
        """
        Generates trajectories from the current particles by propagating each of them.
        Parameters
        ----------
        velocity : float
            Current velocity of the human
        dt : float
            Time step
        num_trajectories : int
            Number of trajectories to generate.
        prediction_time : int
            Time steps to predict into the future.
       """

        trajectories = []
        #print("len(self._estimated_positions: {}".format(len(self._estimated_positions)))
        for i in range(num_trajectories):
            # First, sample a position and a goal from the current belief and the estimated positions
            position, goal = self._sample_position(self._estimated_positions, current_belief)
            
            # Generate a full trajectory
            trajectory = self._generate_trajectory(position, velocity, goal, dt, prediction_time)

            trajectories.append(trajectory)
        return trajectories


    def _generate_trajectory(self, current_pos, velocity, goal, dt, prediction_time):
        """
        Generates a single trajectory up to a certain amount of time steps from the given position.
        Parameters
        ----------
        current_pos :
            Current position of the human.
        velocity :
            Averaged velocity of the human.
        goal :
            Sampled goal of the human.
        dt : float
            Time step
        prediction_time : int
            Amount of time steps to predict into the future.

        Returns
        -------
        np.ndarray : Trajectory
        """

        history = list(np.copy(self._reached_goals))


        out = []
        out.append(np.copy(current_pos))
        first_goal_idx = np.where(self._goals == goal)[0][0]
        selected_goal = goal
        reached_goal = False
        counter_in_goal = 0


        for _ in range(prediction_time):

            # Particle reached selected goal
            # This will continuously chose a next goal, if a particle already reached its predecessor goal

            if np.linalg.norm(current_pos - selected_goal) <= 0.1:
                reached_goal = True



                if counter_in_goal > self.curr_stay_duration_goals[self._is_human_in_range_of_goal(selected_goal)[1]] / dt:

                    selected_goal_idx = np.where(self._goals == selected_goal)[0][0]

                    if len(history) > 0:
                        if not selected_goal_idx == history[-1]:
                            history.append(selected_goal_idx)
                    else:
                        history.append(selected_goal_idx)
                    #print "history:"
                    #print history
                    # Select next goal based on the pre-learned goal-change probabilities


                    #print "selected goal {}".format(selected_goal_idx)
                    probs,available_goals = self.compute_transition_probs_for_goal_from_history(history,selected_goal_idx)


                    for p in probs:
                        if p < self._belief_threshold:
                            p = 0.0

                    print "probs sampling: "
                    print probs / np.sum(np.asarray(probs))
                    selected_goal = self._goals[np.random.choice(available_goals, p=probs / np.sum(np.asarray(probs)))]

                    counter_in_goal = 0.0

                    #print("switching")

                else:
                    counter_in_goal += 1
                    #print("incr counter")



            if reached_goal:
                #print self.curr_stay_duration_goals
                #print self.curr_stay_duration_goals[ self._is_human_in_range_of_goal(selected_goal)[1] ]

                new_pos = self.transition_human(current_pos, velocity, selected_goal, dt)

                out.append(new_pos)
                current_pos = new_pos



            else:
                new_pos = self.transition_human(current_pos, velocity, selected_goal, dt)
                out.append(new_pos)
                current_pos = new_pos

        return np.asarray(out)
    

class ProMPsBeliefComputation(BeliefComputationMethod):
    def __init__(self, belief_tracker, frequency, use_ros, belief_tracker_only, weights_path):
        BeliefComputationMethod.__init__(self, belief_tracker, frequency, use_ros, belief_tracker_only)
        self._weights_path = weights_path
        
        self._weights_means, self._weights_covs, self._alphas_means, self._alphas_covs, self._feature_matrix_parameters = promps.load_learned_weights(self._weights_path)
        self._N_w, self._h_w, self._N_alpha, self._h_alpha = self._feature_matrix_parameters

        self._learn_weights_online = True
        if self._learn_weights_online:
            self._trajectories = defaultdict(list)
            self._weights_means = defaultdict(np.ndarray)
            self._weights_covs = defaultdict(np.ndarray)
            self._ridge_factor_w = 1e-12

        self.current_trajectory = History()
        self._started_moving = False

        self._current_phases = np.empty(0)
        self._predicted_trajectories_means = defaultdict(np.ndarray)
        self._predicted_trajectories_covs = defaultdict(np.ndarray)
        self._num_alpha_samples = 20
        self._avg_num_obs = 20


    def  _compute_belief_from_pose_and_time(self, current_human_pos, current_time):
        self._current_human_pos = current_human_pos
        dt = current_time - self._last_time
        # Update the Goals by calling the GoalTracker if goal tracker is active.
        if not self._belief_tracker_only:
            self._belief_tracker._update_goals()

        if self._history.size() < 1:
            last_obs = current_human_pos
        else:
            last_obs = self._history.last_entry()[1]

        vel, avg_vel = self._compute_velocity(last_obs, current_human_pos, dt)

        self._last_time = current_time
        self._history.add_entry(current_time, current_human_pos, vel, dt)
        self._direction = self._compute_direction(current_human_pos)

        [is_at_goal, goal_index] = self._is_human_in_range_of_goal(np.asarray(current_human_pos))
        # wait till reach the first goal
        if not self._reached_goals:
            if is_at_goal:
                self._reached_goals.append(goal_index)
                print("reached first goal:", goal_index) # debug
                self._init_belief(goal_index)
            else:
                print("waiting to reach the first goal") # debug
                return [], [], avg_vel

        # detect new (sub)trajectory from some goal to another goal
        if self.current_trajectory.size() == 0:
            # wait till we start moving
            if is_at_goal:
                #print("waiting till we start moving") # debug
                return [], [], avg_vel
            self._started_moving = True

        if self._started_moving:
            # Check if we reached the end of the (sub)trajectory from some goal to another goal
            if is_at_goal:
                print("reached goal:", goal_index) # debug
                self._started_moving = False
                self._reached_goals.append(goal_index)

                #self.plot_cumulative_trajectory_error(self.current_trajectory, self._current_phases, self._reached_goals[-2], self._reached_goals[-1])

                if self._learn_weights_online and len(self._reached_goals) > 1:
                    # add current trajectory to trajectories from start goal to target goal
                    start_goal = self._reached_goals[-2]
                    target_goal = self._reached_goals[-1]
                    traj = self.convert_history_to_ndarray_trajectory(self.current_trajectory)
                    key = str(start_goal) + '-' + str(target_goal)
                    self._trajectories[key].append(traj)

                    # update learned weights for trajectories from start goal to target goal
                    print("relearning weights for goal pair trajectories " + key)
                    w_mean, w_cov = promps.learn_weights_distribution(self._trajectories[key], self._N_w, self._h_w, self._ridge_factor_w)
                    self._weights_means[key] = w_mean
                    self._weights_covs[key] = w_cov
                    
                self.current_trajectory.clear()
                self._init_belief(goal_index)
                # TODO: set probability for reached goal to 1 and others to 0
                # TODO: sample trajectories
                sampled_trajectories = []
                return sampled_trajectories, is_at_goal, avg_vel

            # add entry to the current trajectory
            self.current_trajectory.add_entry(current_time, current_human_pos, vel, dt)

            # compute belief for current trajectory
            #print("computing belief...") # debug
            
            new_belief = []
            likelihoods = []
            normalization_factor = 0.0

            # Compute likelihoods and predict trajectories for different target goals
            start_goal = self._reached_goals[-1]
            for target_goal in range(self._num_goals):
                key = str(start_goal) + '-' + str(target_goal)
                key_reversed = str(target_goal) + '-' + str(start_goal)
                # check if key exists in self._weights_means
                if key in self._weights_means and key in self._alphas_means:
                    w_mean = self._weights_means[key]
                    w_cov = self._weights_covs[key]
                    alpha_mean = self._alphas_means[key]
                    alpha_cov = self._alphas_covs[key]
                # otherwise check if we learned weights for trajectories from target goal to start goal and reverse them
                # elif key_reversed in self._weights_means:
                #     rospy.logwarn("using reversed key " + key_reversed)
                #     w_mean = np.flip(self._weights_means[key_reversed], axis=0)
                #     w_cov = np.flip(self._weights_covs[key_reversed], axis=(0,1))
                #     alpha_mean = self._alphas_means[key_reversed]
                #     alpha_cov = self._alphas_covs[key_reversed]
                else:
                    obs_likelihood = 0.00001
                    likelihoods.append(obs_likelihood)
                    continue

                # compute phases
                self._current_phases, alpha_conditional_probs = self.estimate_phases_online(w_mean, w_cov, alpha_mean, alpha_cov)
                
                Phi = promps.compute_feature_matrix(self._current_phases, self._N_w, self._h_w)
                current_Phi = Phi[-1,:]

                # compute observation likelihood
                obs_likelihood = self.compute_observation_likelihood(current_human_pos, 
                                                                    current_Phi, 
                                                                    w_mean, 
                                                                    w_cov)

                normalization_factor += obs_likelihood * self._current_belief[target_goal]
                likelihoods.append(obs_likelihood)

                # compute predicted trajectories mean and cov
                #pred_traj_mean, pred_traj_cov = self.compute_predicted_trajectory_ewerton(alpha_conditional_probs, w_mean, w_cov, alpha_mean, alpha_cov)
                pred_traj_mean, pred_traj_cov = self.compute_predicted_trajectory_dermy(self._current_phases, w_mean, w_cov)
                self._predicted_trajectories_means[target_goal] = pred_traj_mean
                self._predicted_trajectories_covs[target_goal] = pred_traj_cov
                
            
            # Compute new belief
            for target_goal in range(self._num_goals):
                if normalization_factor != 0:
                    prob = (likelihoods[target_goal] * self._current_belief[target_goal]) / normalization_factor
                else:
                    prob = 0.00001
                if prob > 1.0:
                    print("target goal", target_goal)
                    print("normalization factor", normalization_factor)
                    print("likelihood", likelihoods[target_goal])
                    print("current belief", self._current_belief[target_goal])
                    print("prob", prob)
                    prob = 0.00001
                new_belief.append(prob)

            # update current belief
            self._current_belief = new_belief
            print("new belief:", new_belief)

            sampled_trajectories = []
            if np.any(self._current_phases):
                # sample predicted trajectories
                sampled_trajectories = self._generate_trajectories(self._current_phases[-1])

            if self.use_ros:
                # publish belief
                belief_msg = Float64MultiArray()
                belief_msg.data = self._current_belief
                self._belief_tracker._pub_belief.publish(belief_msg)
                
                # publish belief for visualization
                self.publish_belief_visualization()
                
                if sampled_trajectories:
                    # publish sampled trajectories
                    self.publish_sampled_trajectories(sampled_trajectories)
                    # publish sampled trajectories for visualization
                    self._belief_tracker._publish_visualized_trajectories(sampled_trajectories)

            return sampled_trajectories, is_at_goal, avg_vel


    def estimate_phases_online(self, w_mean, w_cov, alpha_mean, alpha_cov):
        # sample alphas from alpha distribution
        num_samples = self._num_alpha_samples
        alpha_sampled = np.random.multivariate_normal(alpha_mean, alpha_cov, num_samples)
        # convert current trajectory to numpy.ndarray
        traj = self.convert_history_to_ndarray_trajectory(self.current_trajectory)
        # compute conditional probabilities of alpha for different samples given current trajectory
        alpha_conditional_prob = np.empty(num_samples)
        for j in xrange(num_samples):
            phases = promps.compute_phases(traj, alpha_sampled[j], self._N_alpha, self._h_alpha)
            alpha_conditional_prob[j], _ = promps.compute_alpha_likelihood(traj, phases, w_mean, w_cov, self._N_w, self._h_w, self._obs_variance)
            #print("alpha conditional", alpha_conditional_prob[j])
        # select alpha with maximum conditional probability
        alpha_star = alpha_sampled[np.argmax(alpha_conditional_prob)]
        phases = promps.compute_phases(traj, alpha_star, self._N_alpha, self._h_alpha)
        #print("estimated phases", phases)
        return phases, alpha_conditional_prob

    
    def compute_observation_likelihood(self, current_observation, current_Phi, w_mean, w_cov):
        mean = np.dot(current_Phi, w_mean)
        var_x = multi_dot([current_Phi, w_cov[:,:,0], current_Phi.T]) + self._obs_variance # or self._human_std_dev**2 ?
        var_y = multi_dot([current_Phi, w_cov[:,:,1], current_Phi.T]) + self._obs_variance
        var_z = multi_dot([current_Phi, w_cov[:,:,2], current_Phi.T]) + self._obs_variance
        obs_cov = np.diag([var_x, var_y, var_z])
        return multivariate_normal.pdf(x=current_observation, mean=mean, cov=obs_cov)


    def convert_history_to_ndarray_trajectory(self, history):
        # convert current trajectory to numpy.ndarray
        timestamps = np.asarray(history.timestamps).reshape(len(history.timestamps), 1)
        positions = np.asarray(history.observations)
        traj = np.concatenate([timestamps, positions], axis=1)
        return traj


    def compute_rest_phases(self, phases):
        n=len(phases)
        z_t=phases[-1]
        if z_t != 0:
            num_rest=int((1-z_t)*n/z_t)
        else:
            num_rest = self._avg_num_obs - 1
        return np.linspace(phases[-1], 1.0, num_rest + 1)[1:]


    def compute_predicted_trajectory_ewerton(self,alpha_conditional_probs,w_mean,w_cov, alpha_mean, alpha_cov):
        num_samples = self._num_alpha_samples
        alpha_sampled = np.random.multivariate_normal(alpha_mean, alpha_cov, num_samples)
        #print("the shape of alpha_conditiona_probs:",alpha_conditional_probs.shape)
        #print ("the probability",alpha_conditional_probs[0])

        # convert current trajectory to numpy.ndarray
        traj = self.convert_history_to_ndarray_trajectory(self.current_trajectory)

        phases = []
       
        for j in xrange(num_samples):
            phases_j = promps.compute_phases(traj, alpha_sampled[j], self._N_alpha, self._h_alpha)
            rest_phases_j = self.compute_rest_phases(phases_j)
            phases.append(np.concatenate((phases_j, rest_phases_j), axis=0))

        # calculate the maximum shape of phases
        M=[]
        for i in xrange(num_samples):
            M.append(len(phases[i]))
        
        m=np.amax(M)
        # make all phases have the same shape
        for j in xrange(num_samples):
            if len(phases[j])<m:
                width=m-len(phases[j])
                phases[j]=np.pad(phases[j],(0,width),"constant",constant_values=1)

    

        # calculate phi  of the whole trajectory j
        # calculate the mean of whole trajectory j
        
        sum_prob_trajectory=0

        for j in xrange(num_samples):

            phi=promps.compute_feature_matrix(phases[j],self._N_w,self._h_w)
            trajectory_mean_j=np.dot(phi,w_mean)
            #print("phases_j",phases[j])
            sum_prob_trajectory= sum_prob_trajectory+np.multiply(alpha_conditional_probs[j],trajectory_mean_j)# the shape problem (8,3) (7,3)
    
        trajectory_mean_predict=sum_prob_trajectory/np.sum(alpha_conditional_probs)   

        # Calculate the covariance of the  trajectory_j
        num_positions = trajectory_mean_predict.shape[0]
        num_dimensions = trajectory_mean_predict.shape[1]
        sum_prob_trajectory_cov = np.zeros([num_positions, num_positions, num_dimensions])

        for j in xrange(num_samples):
            phi = promps.compute_feature_matrix(phases[j], self._N_w, self._h_w)
            H = np.empty([phi.shape[0], phi.shape[0], num_dimensions])
            # compute covariance separately for each dimension
            for i in range(num_dimensions):
                trajectory_mean_j_i = np.dot(phi,w_mean[:,i]).reshape([phi.shape[0], 1])
                trajectory_mean_predict_i = trajectory_mean_predict[:,i].reshape([num_positions, 1])

                trajectory_cov_j = np.diag(np.sum(np.dot(phi, w_cov[:,:,i]) * phi, axis=1) + self._obs_variance)
                H[:,:,i] = trajectory_cov_j + np.dot(trajectory_mean_j_i, trajectory_mean_j_i.T) - np.dot(trajectory_mean_predict_i, trajectory_mean_predict_i.T)

            sum_prob_trajectory_cov +=  alpha_conditional_probs[j] * H 

        trajectory_cov_predict=sum_prob_trajectory_cov /np.sum(alpha_conditional_probs)
    
        return trajectory_mean_predict, trajectory_cov_predict


    def compute_predicted_trajectory_dermy(self, phases, w_mean, w_cov):
        """
         w_mean is Nx3 matrix
         N: the number of basis function

       """
        traj = self.convert_history_to_ndarray_trajectory(self.current_trajectory)
        positions= traj[:,1:]

        # compute the whole phases from Z_0 to Z_max:
        rest_phases = self.compute_rest_phases(phases)
        whole_phases = np.concatenate((phases, rest_phases), axis=0)

        # feature matrix for current trajectory
        phi_ij= promps.compute_feature_matrix(phases, self._N_w, self._h_w)
        # feature matrix for whole (predicted) trajectory
        phi_j = promps.compute_feature_matrix(whole_phases, self._N_w, self._h_w)

        # compute w_mean and w_cov separately for each dimension
        N = w_mean.shape[0]
        num_dimensions = w_mean.shape[1]
        w_mean_new = np.empty([N, num_dimensions])
        w_cov_new = np.empty([N, N, num_dimensions])
        for i in range(num_dimensions):
            C = np.dot(w_cov[:,:,i],phi_ij.T)
            D = np.diag(np.sum(C * phi_ij.T, axis=0) + self._obs_variance)
            b = positions[:,i] - np.dot(phi_ij, w_mean[:,i])
            x = np.linalg.solve(D, b)
            Lb = np.dot(C, x)
            w_mean_new[:,i] = w_mean[:,i] + Lb

            y = np.linalg.solve(D, C.T)
            La = np.dot(C, y)
            w_cov_new[:,:,i] = w_cov[:,:,i] - La

        # compute trajectory mean
        mean_tau = np.dot(phi_j, w_mean_new)
        # compute covariance of trajectory separately for each dimension
        num_positions = mean_tau.shape[0]
        cov_tau = np.empty([num_positions, num_positions, num_dimensions])
        for i in range(num_dimensions):
            cov_tau[:,:,i] = np.diag(np.sum(np.dot(phi_j, w_cov[:,:,i]) * phi_j, axis=1) + self._obs_variance)

        return mean_tau, cov_tau


    def _generate_trajectories(self, last_phase):
        """
        Generates predicted trajectories from current position.

        Parameters
        ----------
        last_phase : float
            Predicted phase of the last observation.
       """

        trajectories = []
        for target_goal in range(self._num_goals):
            # compute how many trajectories to sample (depends on the belief)
            num_samples = int(round(self._num_sampled_trajectories * self._current_belief[target_goal]))
            if num_samples == 0:
                continue
            # check if there is a predicted trajectory mean for the given target goal
            if not target_goal in self._predicted_trajectories_means:
                continue

            mean = self._predicted_trajectories_means[target_goal]
            cov = self._predicted_trajectories_covs[target_goal]

            # check if mean of trajectory is not empty
            if not np.any(mean):
                continue
            
            # cut observed part of the trajectory mean 
            num_positions = len(mean)
            pos_start_idx = int(math.ceil(last_phase * num_positions))
            mean = mean[pos_start_idx:,:]
            cov = cov[pos_start_idx:, pos_start_idx:, :]
            num_positions_new = mean.shape[0]

            # do not sample if new mean is 0
            if not np.any(mean):
                continue

            # sample trajectories
            """ sampled_x = np.random.multivariate_normal(mean[:,0], cov[:,:,0], num_samples).reshape(num_samples, num_positions_new, 1)
            sampled_y = np.random.multivariate_normal(mean[:,1], cov[:,:,1], num_samples).reshape(num_samples, num_positions_new, 1)
            sampled_z = np.random.multivariate_normal(mean[:,2], cov[:,:,1], num_samples).reshape(num_samples, num_positions_new, 1)
            sampled_trajectories = np.concatenate([sampled_x, sampled_y, sampled_z], axis=2) """
            #print("sampled_trajectories shape", sampled_trajectories.shape)

            pos_dim = mean.shape[1]
            sampled_deviation = np.random.multivariate_normal(np.zeros(pos_dim), np.identity(pos_dim), num_samples)
            means = mean.reshape([1, num_positions_new, pos_dim]).repeat(num_samples, axis=0)
            cov_x = np.sqrt(np.diag(cov[:,:,0])).reshape([1, num_positions_new, 1])
            cov_y = np.sqrt(np.diag(cov[:,:,1])).reshape([1, num_positions_new, 1])
            cov_z = np.sqrt(np.diag(cov[:,:,2])).reshape([1, num_positions_new, 1])
            covariances = np.concatenate([cov_x, cov_y, cov_z], axis=2).repeat(num_samples, axis=0)
            sampled_deviations = sampled_deviation.reshape(num_samples, 1, pos_dim).repeat(num_positions_new, axis=1)
            deviations = covariances * sampled_deviations
            sampled_trajectories = means + deviations

            # add them to the list of trajectories
            trajectories += sampled_trajectories.tolist()
            
        return trajectories


    def publish_belief_visualization(self):
        ma = MarkerArray()
        for i in range(self._num_goals):
            m = Marker()
            m.header.frame_id = "darias"
            m.header.stamp = rospy.Time.now()
            m.ns = "my_namespace"
            m.id = i + 1000
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = self._goals[i][0]
            m.pose.position.y = self._goals[i][1]
            m.pose.position.z = self._goals[i][2] + 0.1
            m.text = "{0:.4f}".format(self._current_belief[i])
            m.pose.orientation.w = 1.0
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            ma.markers.append(m)
        self._belief_tracker._pub_belief_vis.publish(ma)


    def publish_sampled_trajectories(self, sampled_trajectories):
        trajectories = PredictedTrajectories()
        for i in xrange(len(sampled_trajectories)):
            path = Path()
            for j in xrange(len(sampled_trajectories[i])):
                p = sampled_trajectories[i][j]
                pose = PoseStamped()
                pose.header.frame_id = "darias"
                pose.header.stamp = rospy.Time(0)
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = p[2]
                path.poses.append(pose)
            trajectories.trajectories.append(path)
        
        self._belief_tracker._pub_sampled_trajectories.publish(trajectories)


    def plot_cumulative_trajectory_error(self, current_trajectory, phases, start_goal, target_goal):
        key = str(start_goal) + '-' + str(target_goal)
        mean_w = self._weights_means[key]
        # check if mean_w is not empty
        if not np.any(mean_w):
            print("no weight mean")
            return

        # convert positions of current trajectory to numpy.ndarray
        positions = np.asarray(current_trajectory.observations)

        # compute mean trajectory from start goal to target goal
        Phi = promps.compute_feature_matrix(phases, self._N_w, self._h_w)
        trajectory_mean = np.dot(Phi, mean_w)
        
        # compute cumulative trajectory error
        error_cum = np.cumsum(np.linalg.norm(positions - trajectory_mean, axis=1))

        fig = plt.figure()
        plt.plot(phases, error_cum)
        plt.draw()
        plt.show()
    