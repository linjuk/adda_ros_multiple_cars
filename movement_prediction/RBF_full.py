import numpy as np
import matplotlib.pyplot as plt
import random
import math
from matplotlib.collections import PatchCollection
from functions import comb_dataset, read_csv_fast

def define_basis_functions(z_vec, N):
    """
    :param T: number of time steps of trajectories
    :param N:  number of basis functions
    :return: matrix of basis functions [N x T --> Nb of basis function x Time steps]
    """

    # Define generic basis function
    cov = 1.0 / (N * N)
    psi = lambda n, t: np.exp(-2 * ((t - (float(n) / (N-1))) ** 2 / cov))

    # Define PSIs matrix
    PSI_matrix = np.zeros((N, len(z_vec)))
    for n in range(N):
        for i in range(len(z_vec)):
            z = z_vec[i]
            PSI_matrix[n, i] = psi(n, z)
    PSI_matrix = PSI_matrix / np.sum(PSI_matrix, 0)
    return PSI_matrix

def learn_weights(norm_data, PSIs_matrix, LAMBDA_COEFF=1e-12):
    """
    :param norm_data: predifined trajectories -> data to learn weights
    :param PSIs_matrix: matrix of basis kernel functions (taken from define_basis_functions)
    :return: learned weights
    """
    # Find out the number of basis functions
    N = PSIs_matrix.shape[0]

    # Find out the dimentionality of trajectories (x and y)
    dof = norm_data[0].shape[1]

    # There is a matrix of zeros (#Dofs x #basis functions) of weights for each of the demonstrations.
    weights = np.zeros((norm_data.shape[0], dof, N))

    # fill weights matrix
    for index in range(norm_data.shape[0]):
        for i in range(dof):
            # In case some regularization is necessary
            # weights[index][i] = np.dot(np.linalg.inv(np.dot(PSIs_matrix, PSIs_matrix.T) + 10e-12 * np.eye(np.dot(PSIs_matrix, PSIs_matrix.T).shape[0])), np.dot(PSIs_matrix, norm_data[index][:,i]))
            weights[index][i] = np.dot(np.linalg.pinv(
                np.dot(PSIs_matrix, PSIs_matrix.T) + LAMBDA_COEFF * np.identity(PSIs_matrix.shape[0])),
                np.dot(PSIs_matrix, norm_data[index][:, i]))

    return weights

# def generate_posterior(norm_data, PSIs_matrix, LAMBDA_COEFF=1e-12):



####################################################
################ Call For Functions ################
####################################################

time_steps = 100
N=5 # Number of basis functions

###### Prepare Matrix for Basis Functions ######

z = np.linspace(-0, 1, time_steps) # starting at 0, ending at 1
a= z[:,None]
psi = define_basis_functions(z, N)

###### Calculate weights ######

dataset = comb_dataset(time_steps)[:3] # get dataset

# calculate weight for each direction
weights_right, weights_left, weights_straight = learn_weights(dataset[0], psi), learn_weights(dataset[1], psi), learn_weights(dataset[2], psi)
# take mean on weights
x_weights_right, y_weights_right  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
x_weights_left, y_weights_left  = np.mean(weights_left[:,0], axis=0), np.mean(weights_left[:,1], axis=0)
x_weights_straight, y_weights_straight  = np.mean(weights_straight[:,0], axis=0), np.mean(weights_straight[:,1], axis=0)

# take variance on weights
# right sigma x and y
for i in range(len(weights_right)):
    sigma_weight_right_x = (sum((weights_right[i,0] - x_weights_right).T * (weights_right[i,0] - x_weights_right))) / (len(weights_right) - 1)
    sigma_weight_right_y = (sum((weights_right[i,1] - y_weights_right).T * (weights_right[i,1] - y_weights_right))) / (len(weights_right) - 1)
# print("sigma weight right [x, y]: ", sigma_weight_right_x, sigma_weight_right_y)
# straight sigma x and y
for i in range(len(weights_straight)):
    sigma_weight_straight_x = (sum((weights_straight[i,0] - x_weights_straight).T * (weights_straight[i,0] - x_weights_straight))) / (len(weights_straight) - 1)
    sigma_weight_straight_y = (sum((weights_straight[i,1] - y_weights_straight).T * (weights_straight[i,1] - y_weights_straight))) / (len(weights_straight) - 1)
# print("sigma weight straight [x, y]: ", sigma_weight_straight_x, sigma_weight_straight_y)
# left sigma x and y
for i in range(len(weights_left)):
    sigma_weight_left_x = (sum((weights_left[i,0] - x_weights_left).T * (weights_left[i,0] - x_weights_left))) / (len(weights_left) - 1)
    sigma_weight_left_y = (sum((weights_left[i,1] - y_weights_left).T * (weights_left[i,1] - y_weights_left))) / (len(weights_left) - 1)
# print("sigma weight left [x, y]: ", sigma_weight_left_x, sigma_weight_left_y)


# Calculation of reconstructed trajectory based on means
reconstr_traj_mean_right_x, reconstr_traj_mean_right_y = np.dot(psi.T, x_weights_right[:,None]),  np.dot(psi.T, y_weights_right[:,None])
reconstr_traj_mean_left_x, reconstr_traj_mean_left_y = np.dot(psi.T, x_weights_left[:,None]), np.dot(psi.T, y_weights_left[:,None])
reconstr_traj_mean_straight_x, reconstr_traj_mean_straight_y = np.dot(psi.T, x_weights_straight[:,None]), np.dot(psi.T, y_weights_straight[:,None])


# Predicting Future Movements from Initial Observations 3.3 chapter

# RIGHT
z_posterior = np.linspace(-0, 1, 17) # starting at 0, ending at 1
psi_posterior = define_basis_functions(z_posterior, N)

observ_right = read_csv_fast('trajectories/test_right_100_initObs.csv') # ~17% of original trajectory [time, x, y, z]
K_right_x = (np.mean(sigma_weight_right_x) * psi_posterior.T) / (0.00001 + psi_posterior.T * np.mean(sigma_weight_right_x) * psi_posterior.T) # 0.00001 - expected trajectory noise
x_sigma_right_posterior = sigma_weight_right_x - np.mean(K_right_x) * (psi_posterior * sigma_weight_right_x)
x_weights_right_posterior = np.mean(x_weights_right) + np.mean(K_right_x) * (np.mean(observ_right[:,1]) - psi_posterior * np.mean(x_weights_right))

# time modulation