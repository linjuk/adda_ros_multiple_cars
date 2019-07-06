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
    phases = (half_timestamp - full_timestamps[0]) / (full_timestamps[-1] - full_timestamps[0])
    return phases

def learn_weights(norm_data, PSIs_matrix, LAMBDA_COEFF=1e-12):
    """
    :param norm_data: predifined trajectories -> data to learn weights
    :param PSIs_matrix: matrix of basis kernel functions (taken from compute_feature_matrix)
    :return: learned weights
    """
    # Find out the number of basis functions
    N = PSIs_matrix.shape[1]
    # Find out the dimentionality of trajectories (x and y)
    dof = norm_data[0].shape[1]
    # There is a matrix of zeros (#Dofs x #basis functions) of weights for each of the demonstrations.
    weights = np.zeros((norm_data.shape[0], dof, N))
    # fill weights matrix
    for index in range(norm_data.shape[0]):
        for i in range(dof):
            # In case some regularization is necessary
            # weights[index][i] = np.dot(np.linalg.inv(np.dot(PSIs_matrix, PSIs_matrix.T) + 10e-12 * np.eye(np.dot(PSIs_matrix, PSIs_matrix.T).shape[0])), np.dot(PSIs_matrix, norm_data[index][:,i]))

            # weights[index][i] = np.dot(np.linalg.pinv(
            #     np.dot(PSIs_matrix, PSIs_matrix.T) + LAMBDA_COEFF * np.identity(PSIs_matrix.shape[0])),
            #     np.dot(PSIs_matrix, norm_data[index][:, i]))

            A = np.dot(PSIs_matrix.T, PSIs_matrix) + LAMBDA_COEFF * np.identity(N)
            B = np.dot(PSIs_matrix.T, norm_data[index][:, i])
            weights[index,i,:] = np.linalg.solve(A, B)

    return weights

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

###### PHASE calculation ###############
interpolated_timestamps = interpolate_timestamps(time_steps)
percentageFULL = 100
part_timestamps = (len(interpolated_timestamps) * (percentageFULL) / 100)
interpolated_timestamps_full = interpolated_timestamps[0:part_timestamps]

phase_full = normalize_time(interpolated_timestamps, interpolated_timestamps_full)
# z = np.linspace(0, 1, 100) # starting at 0, ending at 1
# a= z[:,None]
phases_full = phase_full[:, None]
psi = compute_feature_matrix(phases_full, N, h) # shape (6, 100)

###### Calculate WEIGHTS ######
weights_right = learn_weights(dataset[0], psi)

###### Calculate MEAN of weights ######
weight_mean_right_x, weight_mean_right_y  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
weights_mean_right_x, weights_mean_right_y  = weight_mean_right_x[:,None], weight_mean_right_y[:,None] # shape (6, 1)

combined_weights_mean_xy = [weights_mean_right_x[:, 0], weights_mean_right_y[:, 0]]

###### Reconstructed Trajectory ######
x_weights_right, y_weights_right  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
reconstr_traj_mean_right_x, reconstr_traj_mean_right_y = np.dot(psi, x_weights_right[:,None]).reshape([time_steps]),  np.dot(psi, y_weights_right[:,None]).reshape([time_steps])

###### Calculate COVARIANCE of weights ######
weights_cov_right_x = np.cov(weights_right[:,0].T) # shape (6, 6)
weights_cov_right_y = np.cov(weights_right[:,1].T)
combined_cov_right_xy = [weights_cov_right_x, weights_cov_right_y]

###### bound calculation for mean ######

traj_cov_x_diag = np.sum(psi.dot(weights_cov_right_x) * psi, axis=1)
std_x = np.sqrt(traj_cov_x_diag)
bound_upp_x = reconstr_traj_mean_right_x + 2 * std_x
bound_bottom_x = reconstr_traj_mean_right_x - 2 * std_x

traj_cov_y_diag = np.sum(psi.dot(weights_cov_right_y) * psi, axis=1)
std_y = np.sqrt(traj_cov_y_diag)
bound_upp_y = reconstr_traj_mean_right_y + 2 * std_y
bound_bottom_y = reconstr_traj_mean_right_y - 2 * std_y

fig, ax = plt.subplots()
plt.figure(1)
plt.plot(reconstr_traj_mean_right_x, 'black')
plt.fill_between(np.arange(time_steps), bound_upp_x, bound_bottom_x, alpha = 0.5, color = 'red', linewidth = 1)
plt.show()


###########################################################
###### Calculate of NEW MEAN & COVARIANCE | 3rd step ######
###########################################################

test_traj_right = np.mean(dataset[0], axis=0) # mean trajectory from RIGHT DB
percentage = 10
part = (len(test_traj_right) * (percentage) / 100)
test_traj_right_NOTfull = test_traj_right[0:part]

###### PHASE calculation ###############
interpolated_timestamps = interpolate_timestamps(time_steps)
part_timestamps = (len(interpolated_timestamps) * (percentage) / 100)
interpolated_timestamps_NOTfull = interpolated_timestamps[0:part_timestamps]
# print interpolated_timestamps_NOTfull

phase_NOTfull = normalize_time(interpolated_timestamps, interpolated_timestamps_NOTfull)
# print phase_NOTfull
# exit(1)

# phases = []
# for t in range(len(interpolated_timestamps_NOTfull)):
#     phase = normalize_time(interpolated_timestamps, interpolated_timestamps_NOTfull)
#     phases.append(phase)
#
# print phases
# # print phase
# # exit(1)

# feature matrix for current trajectory
N1 = 8
h1 = 0.1
obs_variance = 0.0005 #????

z1 = np.linspace(0, 1, len(test_traj_right_NOTfull)) # starting at 0, ending at 1
psi_new = compute_feature_matrix(phase_NOTfull, N1, h1) # shape (6, 10)
# print "shape: ", psi_new.shape
psi_mean = []
for a in range(len(psi_new)):
    psi_mean.append(np.mean(psi_new[a]))

# compute w_mean and w_cov separately for each dimension
num_dimensions = 2
w_mean_new = [np.empty([N1]) for i in range(num_dimensions)]
w_cov_new = [np.empty([N1, N1]) for i in range(num_dimensions)]


for i in range(num_dimensions): # for BOTH DIMENTIONS

    C = np.dot(combined_cov_right_xy[i], psi_new.T)                                  # shape (8,10)
    D = np.diag(np.sum(C * psi_new.T, axis=0) + obs_variance)                        # shape (10, 10)
    b = test_traj_right_NOTfull[:, i] - np.dot(psi_new, combined_weights_mean_xy[i]) # shape 10
    x = np.linalg.solve(D, b)                                                        # shape (10, 8)
    Lb = np.dot(C, x)                                                                # shape 8
    w_mean_new[ i] = combined_weights_mean_xy[i] + Lb
    y = np.linalg.solve(D, C.T)                                                      # shape (10, 8)
    La = np.dot(C, y)                                                                # shape (8, 8)
    w_cov_new[i] = np.array(combined_cov_right_xy[i]) - La

    # print "COV1: ", combined_cov_right_xy[i].shape
    # print "C: ", C.shape
    # print "D : ", D.shape
    # print "meanALL: ", combined_weights_mean_xy[i].shape
    # print "b: ", b.shape
    # print "x: ", x.shape
    # print "y: ", y.shape
    # print "Lb: ", Lb.shape
    # print "La: ", La.shape


###### Reconstructed NOT FULL Trajectory ######
reconstr_traj_mean_right_x_new, reconstr_traj_mean_right_y_new = np.dot(psi_new, w_mean_new[0]),  np.dot(psi_new, w_mean_new[1])

###### Reconstructed PREDICTED FULL Trajectory ######
reconstr_traj_mean_right_x_PREDICTED, reconstr_traj_mean_right_y_PREDICTED = np.dot(psi, w_mean_new[0]),  np.dot(psi, w_mean_new[1])


# print len(reconstr_traj_mean_right_x), len(reconstr_traj_mean_right_y)
# print len(reconstr_traj_mean_right_x_new), len(reconstr_traj_mean_right_y_new)

# print "full:", test_traj_right
# print "NOT_full:", test_traj_right_NOTfull
# print  w_mean_new, len( w_mean_new)
############################################
################# PLOTTING #################
############################################

plt.title('')
plt.xlabel('x, [m]')
plt.ylabel('y, [m]')

labels = {
        'real': 'Mean Trajectory from Demonstations',
        'reconstructed': 'Reconstructed trajectory, using mean of weigts',
        'reconstructedNOTFULL': 'Reconstructed Not Full trajectory',
        'reconstructedFULL': 'Reconstructed  PRedicted Full trajectory'
}

plt.plot()
plt.plot(test_traj_right[:, 0], test_traj_right[:, 1], 'blue', label=labels['real'])
plt.plot(reconstr_traj_mean_right_x, reconstr_traj_mean_right_y, 'red', label=labels['reconstructed'])
plt.plot(reconstr_traj_mean_right_x_new, reconstr_traj_mean_right_y_new, '--go', label=labels['reconstructedNOTFULL'])
plt.plot(reconstr_traj_mean_right_x_PREDICTED, reconstr_traj_mean_right_y_PREDICTED, 'black', label=labels['reconstructedFULL'])


plt.legend(loc='lower right')
plt.show()