#!/usr/bin/python2

import numpy as np
from numpy.linalg import multi_dot
#from scipy.stats import multivariate_normal
#import mixpromp as promp

import pickle
import sys
from argparse import ArgumentParser
import os
from collections import defaultdict

import time # for performance analysis

import rospkg
from scipy.stats import multivariate_normal

# plot
import plot_data
import matplotlib.pyplot as plt


def normalize_time(timestamps):
    """
    Computes phase from given timestamps. Phase is normalized time from 0 to 1.
    """
    phases = (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0])
    return phases


def compute_feature_matrix(phases, N, h):
    """
    Compute a TxN matrix of features of the given phase vector using Gaussian basis functions.
    Where T is the number of elements in the phase vector and N is the number of basis functions.

    Parameters
    ----------
    numpy.ndarray
        phases: vector with phases
    int
        N: number of basis functions in the resulting matrix
    float
        h: width of a basis function (variance)

    Returns
    -------
    numpy.ndarray
        TxN matrix of Gaussian basis functions
    """
    T = len(phases)
    # Uniformly distribute the centers of N basis functions in domain[-2h,2h+1].
    centers = np.linspace(-2 * h, 1 + 2 * h, num=N)
    # compute a TxN matrix with centers
    C = np.repeat(centers.reshape(1, N), T, axis=0)
    # compute a TxN matrix with phases
    P = np.repeat(phases.reshape(T, 1), N, axis=1)
    # compute a TxN feature matrix
    Phi = np.exp(- 0.5 / h * np.square(P - C))
    # normalize the feature matrix
    Phi = Phi / np.sum(Phi, axis=1).reshape(T, 1)
    return Phi


def compute_learned_trajectories(weights_means, weights_covs, phases, Phi, observation_variance, num_goals):
    learned_trajectories = defaultdict(np.ndarray)
    traj_covariances = defaultdict(np.ndarray)
    
    for key, _ in weights_means.iteritems():
        w_mean = weights_means[key]
        # check if w_mean is not empty
        if not np.any(w_mean):
            continue
        w_cov = weights_covs[key]
        # compute positions of the learned trajectory and its covariance
        positions = np.dot(Phi, w_mean)
        # compute covariance matrices for each dimension separately
        num_positions = positions.shape[0]
        num_dimensions = positions.shape[1]
        covariances = np.empty((num_positions, num_positions, num_dimensions))
        for i in range(num_dimensions):
            cov = np.dot(np.dot(Phi, w_cov[:,:,i]), Phi.T) + observation_variance * np.identity(num_positions)
            covariances[:,:,i] = cov

        # add positions and timestamps to the trajectory
        trajectory = np.concatenate((phases.reshape(len(phases), 1), positions), axis=1)
        # save trajectory positions and covariances
        learned_trajectories[key] = trajectory
        traj_covariances[key] = covariances
    
    return learned_trajectories, traj_covariances


def learn_weights_distribution(trajectories, N, h, ridge_factor, compute_phases=True):
    num_trajectories = len(trajectories)
    M = len(trajectories[0][0]) - 1 # dimensionality of y_i (3 for a position)
    W = np.empty((N, M, num_trajectories)) # container with all weights
    for i, traj in enumerate(trajectories):
        if compute_phases:
            phases = normalize_time(traj[:,0])
        else:
            # make timestamps (saved in phases) start from 0
            phases = traj[:,0] - traj[0,0]
        positions = traj[:,1:]
        Phi = compute_feature_matrix(phases, N, h)
        # learn weights matrix
        A = np.dot(Phi.T, Phi) + ridge_factor * np.identity(N)
        B = np.dot(Phi.T, positions)
        weights = np.linalg.solve(A, B)
        # make sure that the weights fulfil the equation system
        #assert(np.allclose(np.dot(A, weights), B))
        # save weights matrix in the weights container
        W[:,:,i] = weights
    # compute weights mean and covariance
    w_mean = np.sum(W, axis=2) / num_trajectories
    W_diff = W - np.repeat(w_mean.reshape(N, M, 1), num_trajectories, axis=2)

    w_cov = np.empty((N, N, M))
    # compute covariance matrices separately for every dimension
    for i in range(M):
        W_diff_i = W_diff[:,i,:]
        w_cov[:,:,i] = np.tensordot(W_diff_i, W_diff_i, axes=(1, 1)) / num_trajectories

    # if we only have one dimension for y_i, we can reduce the dimensionality of w_cov to NxN
    if M == 1:
        w_cov = w_cov.reshape((N, N))

    return w_mean, w_cov


def learn_trajectories(trajectories, N, h, ridge_factor, N_alpha, h_alpha, ridge_factor_alpha, num_samples, observation_variance, learn_alphas = False):
    num_goals = len(trajectories[0])
    # weights_means = [[np.empty(0) for _ in range(num_goals)] for _ in range(num_goals)]
    # weights_covs = [[np.empty(0) for _ in range(num_goals)] for _ in range(num_goals)]
    # alphas_means = [[np.empty(0) for _ in range(num_goals)] for _ in range(num_goals)]
    # alphas_covs = [[np.empty(0) for _ in range(num_goals)] for _ in range(num_goals)]

    weights_means = defaultdict(np.ndarray)
    weights_covs = defaultdict(np.ndarray)
    alphas_means = defaultdict(np.ndarray)
    alphas_covs = defaultdict(np.ndarray)

    alpha_means_history_all = defaultdict(list)

    for key, goal_pair_trajectories in trajectories.iteritems():
        # check if the list of trajectories is not empty
        if not goal_pair_trajectories:
            continue
        
        print("goal pair:", key)
        # learn weights and compute their mean and covariance
        w_mean, w_cov = learn_weights_distribution(goal_pair_trajectories, N, h, ridge_factor)
        # save mean and covariance of learned weights
        weights_means[key] = w_mean
        weights_covs[key] = w_cov

        if learn_alphas:
            # learn mean and covariance of alpha
            alpha_mean, alpha_cov, alpha_means_history = learn_alpha_distribution_EM(goal_pair_trajectories, w_mean, w_cov, N, h, N_alpha, h_alpha, ridge_factor_alpha, num_samples, observation_variance)
            # save mean and covariance of learned alphas
            alphas_means[key] = alpha_mean
            alphas_covs[key] = alpha_cov

            alpha_means_history_all[key] = alpha_means_history

    return weights_means, weights_covs, alphas_means, alphas_covs, alpha_means_history_all


def initialize_alpha_distribution(trajectories, N, h, ridge_factor):
    # fill velocities vector
    velocities = []
    for i, traj in enumerate(trajectories):
        values = np.empty([len(traj), 2])
        values[:,0] = traj[:,0]
        avg_velocity = 1 / (values[-1,0] - values[0,0])
        values[:,1] = avg_velocity * np.ones([len(traj)])
        velocities.append(values)

    # learn alpha distribution using ridge regression
    alpha_mean, alpha_cov = learn_weights_distribution(velocities, N, h, ridge_factor, compute_phases=False)

    # TODO: What to do if there is only one trajectory?

    return alpha_mean.reshape(len(alpha_mean)), alpha_cov


def compute_phases(trajectory, alpha, N_alpha, h_alpha):
    timestamps = trajectory[:,0] - trajectory[0,0]
    Phi_alpha = compute_feature_matrix(timestamps[:-1], N_alpha, h_alpha)
    phase_velocities = np.dot(Phi_alpha, alpha)
    #print("phase velocities: ", phase_velocities)
    dt_vec = np.diff(timestamps)
    phases = np.insert(np.cumsum(phase_velocities * dt_vec), 0, 0)
    # phases cannot have values greater than Z
    phases[phases > 1] = 1.0
    return phases


def compute_alpha_likelihood(trajectory, phases, w_mean, w_cov, N, h, observation_variance):
    # compute mean and covariance of learned trajectory
    Phi = compute_feature_matrix(phases, N, h)
    traj_mean = np.dot(Phi, w_mean)    
    # stack mean of trajectory
    traj_mean_stacked = np.concatenate([traj_mean[:,0], traj_mean[:,1], traj_mean[:,2]], axis=0)
    # compute covariance matrix of stacked mean of trajectory
    traj_cov_x_diag = np.sum(Phi.dot(w_cov[:,:,0]) * Phi, axis=1)
    traj_cov_y_diag = np.sum(Phi.dot(w_cov[:,:,1]) * Phi, axis=1)
    traj_cov_z_diag = np.sum(Phi.dot(w_cov[:,:,2]) * Phi, axis=1)
    traj_cov_stacked = np.diag(np.concatenate([traj_cov_x_diag, traj_cov_y_diag, traj_cov_z_diag], axis=0))
    traj_cov_stacked += observation_variance * np.identity(traj_cov_stacked.shape[0])
    # compute likelihood and loglikelihood of trajectory
    trajectory_stacked = np.concatenate([trajectory[:,1], trajectory[:,2], trajectory[:,3]])
    distribution = multivariate_normal(mean=traj_mean_stacked, cov=traj_cov_stacked)
    alpha_likelihood = distribution.pdf(x=trajectory_stacked)
    alpha_log_likelihood = distribution.logpdf(x=trajectory_stacked)

    return alpha_likelihood, alpha_log_likelihood


def learn_alpha_distribution_EM(trajectories, w_mean, w_cov, N, h, N_alpha, h_alpha, ridge_factor_alpha, num_samples, observation_variance):
    num_trajectories = len(trajectories)
    alpha_expectations = np.empty([num_trajectories, N_alpha])
    alpha_sampled = np.empty([num_samples, N_alpha])
    alpha_conditional_prob = np.empty([num_trajectories, num_samples])
    alpha_log_likelihood = np.empty([num_trajectories, num_samples])
    likelihood_total_old = -1000000000
    converged = False
    convergence_threshold = 0.01
    alpha_mean_old = np.zeros(N_alpha)
    # for performance analysis
    alpha_means_history = []

    alpha_mean, alpha_cov = initialize_alpha_distribution(trajectories, N_alpha, h_alpha, ridge_factor_alpha)
    
    while not converged:
    #for i in range(7):
        # sample alphas from alpha distribution
        alpha_sampled = np.random.multivariate_normal(alpha_mean, alpha_cov, num_samples)

        # E step:
        # fill the matrix with conditional probabilities of alpha
        for i, traj in enumerate(trajectories):
            """ # make timestamps start from 0
            timestamps = traj[:,0] - traj[0,0]
            Phi = compute_feature_matrix(timestamps, N_alpha, h_alpha) """
            for j in xrange(num_samples):
                phases = compute_phases(traj, alpha_sampled[j,:], N_alpha, h_alpha)
                #print("alpha_j", alpha_sampled[j])
                #print("estimated phases:", phases)
                alpha_conditional_prob[i,j], alpha_log_likelihood[i,j] = compute_alpha_likelihood(traj, phases, w_mean, w_cov, N, h, observation_variance)

        alpha_conditional_prob_sum = np.sum(alpha_conditional_prob, axis=1).reshape(num_trajectories, 1)
        # repeate alpha_sampled for all trajectories
        alpha_sampled_rep = np.repeat(alpha_sampled.reshape(1, num_samples, N_alpha), num_trajectories, axis=0)
        # repeate alpha_conditional for element wise multiplication with alpha_sampled_rep
        alpha_conditional_rep = np.repeat(alpha_conditional_prob.reshape(num_trajectories, num_samples, 1), N_alpha, axis=2)
        alpha_expectations = np.sum(alpha_conditional_rep * alpha_sampled_rep, axis=1) / alpha_conditional_prob_sum

        # M step:
        alpha_mean = np.sum(alpha_expectations, axis=0) / num_trajectories
        diff = alpha_sampled - alpha_mean
        diff_outer = diff[:,:,None] * diff[:,None,:]
        # repeate diff_outer for all trajectories
        diff_outer = np.repeat(diff_outer.reshape(1, num_samples, N_alpha, N_alpha), num_trajectories, axis=0)
        # repeat alpha_conditional_prob for element wise multiplication with diff_outer
        prob_weights = np.repeat(alpha_conditional_prob.reshape(num_trajectories, num_samples, 1, 1), N_alpha * N_alpha, axis=2).reshape(num_trajectories, num_samples, N_alpha, N_alpha)
        # compute weighted diff summ
        weighted_diff_sum = np.sum(prob_weights * diff_outer, axis=(0, 1))
        alpha_cov = weighted_diff_sum / np.sum(alpha_conditional_prob_sum)

        # compute termination decision
        likelihood_total = np.sum(alpha_log_likelihood, axis=(0,1))
        likelihood_diff = likelihood_total - likelihood_total_old
        converged = likelihood_diff <= convergence_threshold
        #print("log_likelihood total:", likelihood_total)
        #print("likelihood_total difference", likelihood_diff)
        #print("alpha_mean", alpha_mean)
        likelihood_total_old = likelihood_total

        normalized_mean_diff = np.linalg.norm(alpha_mean - alpha_mean_old) / len(alpha_mean)
        converged = normalized_mean_diff <= convergence_threshold
        #print("normalized difference of means", normalized_mean_diff)
        alpha_mean_old = alpha_mean

        # save alpha_mean (for performance analysis)
        alpha_means_history.append(alpha_mean)

    return alpha_mean, alpha_cov, alpha_means_history


def save_learned_weights(weights_means, weights_covs, alphas_means, alphas_covs, parameters, traj_path):
    data = defaultdict()
    data["w_means"] = weights_means
    data["w_covs"] = weights_covs
    data["alpha_means"] = alphas_means
    data["alpha_covs"] = alphas_covs
    data["parameters"] = parameters
    #pkg_dirname = os.path.dirname(os.path.abspath('belief_tracker.__file__'))
    pkg_dirname = rospkg.RosPack().get_path('belief_tracker')
    weights_dirname = pkg_dirname + '/data/learned_weights/'
    filename = weights_dirname + os.path.basename(traj_path).replace('_traj.pkl', '') + '_weights.pkl'
    with open(filename, 'wb') as handle:
        pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)


def load_learned_weights(filename):
    with open(filename, 'rb') as handle:
        data = pickle.load(handle)
    return data["w_means"], data["w_covs"], data["alpha_means"], data["alpha_covs"], data["parameters"]


def load_trajectories(filename):
    with open(filename, 'rb') as handle:
        trajectories = pickle.load(handle)
    return trajectories


def load_goals(filename):
    return np.genfromtxt(filename, dtype=float, delimiter=",",skip_header=1)[:,0:3]


def merge_goal_pair_trajectories(trajectories):
    """
    Merge the lists of trajectories from start goal i to target goal j
    and from start goal j to target goal i while reversing the later ones.
    """
    num_goals = len(trajectories[0])
    for start_goal in range(num_goals):
        for target_goal in range(0, start_goal):
            # reverse positions of trajectories
            reverse_positions = lambda x: np.concatenate((x[:,0].reshape(len(x), 1), np.flip(x[:,1:], axis=0)), axis=1)
            #reverse_all = lambda x: np.flip(x, axis=0) # why can we learn although we flip times as well
            reversed_trajectories = list(map(reverse_positions, trajectories[start_goal][target_goal]))
            # add reversed trajectories (from goal j to goal i) to trajectories (from goal i to goal j)
            trajectories[target_goal][start_goal] += reversed_trajectories
            # clear trajectories (from goal j to goal i)
            trajectories[start_goal][target_goal] = []
    
    return trajectories           


def _read_console_args():
    """
    Reads console argument for the path of the pickle file with saved trajectories between goals
    To be used only in main() method.

    Returns
    -------
    string
        Path of the file with saved trajectories between goals.
    """
    parser = ArgumentParser()
    parser.add_argument("-t", "--traj", dest="traj_path",
                        help="path to the pickle file with saved trajectories between goals",
                        default="/home/kinecat/ias_ros/src/intention_aware_promps/belief_tracker/data/trajectories_between_goals/H14L_no_robot_traj.pkl")    
    parser.add_argument("-g", "--goals", dest="goals_path",
                        help="path to goals file",
                        default="/home/kinecat/ias_ros/src/intention_aware_promps/belief_tracker/recorded_data/userstudy_all/H14L/goals_H14L.csv")
    args = parser.parse_args()
     # accept only if entered both paths or neither
    if len(sys.argv) != 1 and len(sys.argv) != 5:
        print("\nenter paths for both trajectory and goals files\n")
        parser.print_help()
        sys.exit()
    return args.traj_path, args.goals_path


def main():
    traj_path, goals_path = _read_console_args()
    trajectories = load_trajectories(traj_path)
    goals = load_goals(goals_path)
    num_goals = len(goals)
    learn_alphas = False

    # Parameters
    N = 4
    h = 0.2
    ridge_factor = 1e-12
    observation_variance = 0.0005 #0.002 #0.0005
    N_alpha = 5
    h_alpha = 0.04
    ridge_factor_alpha = 1e-12
    num_samples = 100

    # set random seed
    random_seed = 19940704
    np.random.seed(random_seed)

    # call if the order of starting and target goal in goal pairs does not matter
    #merge_goal_pair_trajectories(trajectories)

    weights_means, weights_covs, alphas_means, alphas_covs, alpha_means_history_all = learn_trajectories(trajectories, N, h, ridge_factor, N_alpha, h_alpha, ridge_factor_alpha, num_samples, observation_variance, learn_alphas=learn_alphas)
    #print(weights_means[0][1].shape)
    if learn_alphas:
        save_learned_weights(weights_means, weights_covs, alphas_means, alphas_covs, [N, h, N_alpha, h_alpha], traj_path)


    if learn_alphas:
        # plot error between estimated and real phases for trajectories 
        # from start goal to target goal in different iterations of EM
        plot_data.plot_phases_error_EM(trajectories, alpha_means_history_all, N_alpha, h_alpha)


    # plot learned trajectories
    phases = np.linspace(0, 1, 100)
    Phi = compute_feature_matrix(phases, N, h)
    learned_trajectories, traj_covariances = compute_learned_trajectories(weights_means, weights_covs, phases, Phi, observation_variance, num_goals)
    ax1 = plot_data.plot_all_trajectories(learned_trajectories)
    #ax2 = plot_data.plot_trajectories_between_goals(learned_trajectories, 0, 3)
    #ax3 = plot_data.plot_trajectories_between_goals(learned_trajectories, 3, 0)
    #axes = (ax1, ax2, ax3)
    axes = ax1
    plot_data.plot_goals(goals, axes)
    
    # plot basis functions
    #plot_data.plot_basis_functions(phases, Phi)

    # plot learned trajectories with variances for axes x, y, z separately
    #plot_data.plot_learned_trajectories_xyz(learned_trajectories, traj_covariances)

    plt.show()

    ### pos_cov = np.dot(np.dot(Phi, w_cov), Phi.T)

    return


if __name__ == '__main__':
    main()