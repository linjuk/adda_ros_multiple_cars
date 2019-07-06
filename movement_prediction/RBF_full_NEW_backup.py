import numpy as np
import matplotlib.pyplot as plt
from functions import comb_dataset, read_csv_fast
import os
import pandas as pd

def interpolate_timestamps(NUM_POINTS):
    csv_data = read_csv_fast(os.path.dirname(os.path.realpath(__file__))+'/trajectories/right_100.csv')
    timestamps = csv_data[:,0]
    duration = timestamps[-1] - timestamps[0]
    interpolated_duration_list = [0]

    for i in range(NUM_POINTS-2):
        interpolated_duration_list.append(np.nan)

    interpolated_duration_list.append(duration)
    series = pd.Series(interpolated_duration_list)
    result = series.interpolate()
    return np.array(result)

def normalize_time(full_timestamps, half_timestamp):
    """
    Computes phase from given timestamps. Phase is normalized time from 0 to 1.
    """
    phases = (half_timestamp[-1] - full_timestamps[0]) / (full_timestamps[-1] - full_timestamps[0])
    return phases

def define_basis_functions(z_vec, N, cov):
    """
    :param T: number of time steps of trajectories
    :param N:  number of basis functions
    :return: matrix of basis functions [N x T --> Nb of basis function x Time steps]
    """

    # Define generic basis function
    # cov = 1.0 / (N * N)
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

            # OR
            # A = np.dot(Phi.T, Phi) + LAMBDA_COEFF * np.identity(N)
            # B = np.dot(Phi.T, positions)
            # weights = np.linalg.solve(A, B)

    return weights

####################################################
################ Call For Functions ################
####################################################

# parameters
N = 8 # Number of basis functions
h = 0.1 #1.0 / (N * N)
ridge_factor = 1e-12
time_steps = 100

dataset = comb_dataset(time_steps)[:3] # get dataset
num_right_traj = len(dataset[0]) # number of right trajectories in dataset
M = 2 # dimentionality

###### Prepare Matrix for Basis Functions ######

z = np.linspace(0, 1, time_steps) # starting at 0, ending at 1
a= z[:,None]
psi = define_basis_functions(z, N, h) # shape (6, 100)

###### Calculate WEIGHTS ######
weights_right = learn_weights(dataset[0], psi)

print weights_right.shape
###### Calculate MEAN of weights ######
weight_mean_right_x, weight_mean_right_y  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
weights_mean_right_x, weights_mean_right_y  = weight_mean_right_x[:,None], weight_mean_right_y[:,None] # shape (6, 1)

combined_weights_mean_xy = [weights_mean_right_x[:, 0], weights_mean_right_y[:, 0]]

print weight_mean_right_x.shape
###### Calculate COVARIANCE of weights ######
weights_cov_right_x = np.cov(weights_right[:,0].T) # shape (6, 6)
weights_cov_right_y = np.cov(weights_right[:,1].T)

combined_cov_right_xy = [weights_cov_right_x[:, 0], weights_cov_right_y[:, 0]]
###### Reconstructed Trajectory ######
x_weights_right, y_weights_right  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
reconstr_traj_mean_right_x, reconstr_traj_mean_right_y = np.dot(psi.T, x_weights_right[:,None]),  np.dot(psi.T, y_weights_right[:,None])

###########################################################
###### Calculate of NEW MEAN & COVARIANCE | 3rd step ######
###########################################################

test_traj_right = np.mean(dataset[0], axis=0) # mean trajectory from RIGHT DB
percentage = 9
part = (len(test_traj_right) * (percentage) / 100)
test_traj_right_NOTfull = test_traj_right[0:part]

###### PHASE calculation ###############
interpolated_timestamps = interpolate_timestamps(time_steps)
part_timestamps = (len(interpolated_timestamps) * (percentage) / 100)
interpolated_timestamps_NOTfull = interpolated_timestamps[0:part_timestamps]

phase = normalize_time(interpolated_timestamps, interpolated_timestamps_NOTfull)

# feature matrix for current trajectory
N1 = 8
h1 = 0.1
obs_variance = 0.0005 #????

z1 = np.linspace(0, 1, len(test_traj_right_NOTfull)) # starting at 0, ending at 1
psi_new = define_basis_functions(z1, N1, h1) # shape (6, 10)
psi_mean = []
for a in range(len(psi_new)):
    psi_mean.append(np.mean(psi_new[a]))

# compute w_mean and w_cov separately for each dimension
num_dimensions = 2
w_mean_new = np.empty([N1, num_dimensions])
w_cov_new = np.empty([N1, N1, num_dimensions])

for i in range(num_dimensions): # for BOTH DIMENTIONS

    C = np.dot(weights_cov_right_x[:, i], psi_new)
    D = np.diag(np.sum(C * psi_new, axis=0) + obs_variance)
    b = test_traj_right_NOTfull[:, i] - np.dot(psi_mean, combined_weights_mean_xy[i] )
    x = np.linalg.solve(D, b)
    Lb = np.dot(C, x)
    w_mean_new[:, i] = combined_weights_mean_xy[i] + Lb
    y = np.linalg.solve(D, C.T)
    La = np.dot(C, y)
    w_cov_new[:, :, i] = np.array(combined_cov_right_xy[i]) - La

###### Reconstructed NOT FULL Trajectory ######
reconstr_traj_mean_right_x_new, reconstr_traj_mean_right_y_new = np.dot(psi_new.T, w_mean_new[:,0]),  np.dot(psi_new.T, w_mean_new[:,1])


print len(reconstr_traj_mean_right_x), len(reconstr_traj_mean_right_y)
print len(reconstr_traj_mean_right_x_new), len(reconstr_traj_mean_right_y_new)

# print "full:", test_traj_right
# print "NOT_full:", test_traj_right_NOTfull
print  w_mean_new, len( w_mean_new)
############################################
################# PLOTTING #################
############################################

plt.title('')
plt.xlabel('x, [m]')
plt.ylabel('y, [m]')

labels = {
        'real': 'Mean Trajectory from Demonstations',
        'reconstructed': 'Reconstructed trajectory, using mean of weigts',
        'reconstructedNOTFULL': 'Reconstructed Not Full trajectory'
}

plt.plot()
plt.plot(test_traj_right[:, 0], test_traj_right[:, 1], 'blue', label=labels['real'])
plt.plot(reconstr_traj_mean_right_x, reconstr_traj_mean_right_y, 'red', label=labels['reconstructed'])
plt.plot(reconstr_traj_mean_right_x_new, reconstr_traj_mean_right_y_new, '--bo', label=labels['reconstructedNOTFULL'])


plt.legend(loc='lower right')
plt.show()
