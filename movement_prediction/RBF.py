import numpy as np
import matplotlib.pyplot as plt
import random
import math
from scipy.spatial import distance
from matplotlib.collections import PatchCollection
from functions import comb_dataset

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

def shadedErrorBar(x, y, errBar, color=None, transparent=0, ax=None):

    y = np.asarray(y).reshape(x.shape)

    # get shaded color
    shaded_color = '#dddddd'

    # plot main line
    ax.plot(x,y, color=color, linewidth=0.8)

    # plot error
    error_up = y + errBar
    error_down = y - errBar
    ax.plot(x,error_up, color=shaded_color)
    ax.plot(x,error_down, color=shaded_color)

    if transparent==1:
        if color is None:
            ax.fill_between(x, error_up, error_down, color=shaded_color, alpha=0.25)
        else:
            ax.fill_between(x,error_up,error_down, color=color, alpha=0.25)
    else:
        if color is None:
            ax.fill_between(x, error_up, error_down, color=shaded_color, alpha=0.25)
        else:
            ax.fill_between(x,error_up,error_down, color=color)
# FIX THIS
def err_on_basis_one_traj_err1 (N, time_steps, selected_direction, random_trajectory_count):
    ###### Prepare Matrix for Basis Functions ######
    z = np.linspace(-0, 1, time_steps)  # starting at 0, ending at 1
    a = z[:, None]
    psi = define_basis_functions(z, N)

    dataset = comb_dataset(time_steps)[:3]  # get dataset

    len_trajectory = [len(dataset[0]), len(dataset[1]), len(dataset[2])]
    # NOT TO CHOSE 1
    selected_direction # 0 = right,  1 = straight, 2 = left
    random_trajectory_count  # randomly selets chosen number of trajectories

    for i in range(0, random_trajectory_count):
        random_trajectory_index = random.randint(0, len_trajectory[selected_direction])

        traj_x = dataset[selected_direction][random_trajectory_index][:, 0, None]
        traj_y = dataset[selected_direction][random_trajectory_index][:, 1, None]

        weights = learn_weights(np.array(dataset[selected_direction]),
                                psi)  # taking weights from all data set per direction
        w_x = weights[i, 0]
        w_y = weights[i, 1]

        reconstr_traj_x = np.dot(psi.T, w_x[:, None])
        reconstr_traj_y = np.dot(psi.T, w_y[:, None])

        err_x = math.sqrt(np.mean((reconstr_traj_x - traj_x) ** 2))
        err_y = math.sqrt(np.mean((reconstr_traj_y - traj_y) ** 2))
        print("Error [x]: ", err_x)
        print("Error [y]: ", err_y)

    return  err_x, err_y

def err_on_basis_mean_traj_err1 (N, time_steps):
    ###### Prepare Matrix for Basis Functions ######
    z = np.linspace(-0, 1, time_steps)  # starting at 0, ending at 1
    a = z[:, None]
    psi = define_basis_functions(z, N)

    dataset = comb_dataset(time_steps)[:3]  # get dataset

    means_right = np.mean(dataset[0], axis=0)
    means_straight = np.mean(dataset[1], axis=0)
    means_left = np.mean(dataset[2], axis=0)

    # calculate weight for each direction
    weights_right, weights_left, weights_straight = learn_weights(dataset[0], psi), learn_weights(dataset[1], psi), learn_weights(dataset[2], psi)

    # take mean on weights
    x_weights_right, y_weights_right = np.mean(weights_right[:, 0], axis=0), np.mean(weights_right[:, 1], axis=0)
    x_weights_left, y_weights_left = np.mean(weights_left[:, 0], axis=0), np.mean(weights_left[:, 1], axis=0)
    x_weights_straight, y_weights_straight = np.mean(weights_straight[:, 0], axis=0), np.mean(weights_straight[:, 1], axis=0)

    # Calculation of reconstructed trajectory based on means
    reconstr_traj_mean_right_x, reconstr_traj_mean_right_y = np.dot(psi.T, x_weights_right[:, None]), np.dot(psi.T, y_weights_right[:, None])
    reconstr_traj_mean_left_x, reconstr_traj_mean_left_y = np.dot(psi.T, x_weights_left[:, None]), np.dot(psi.T, y_weights_left[:, None])
    reconstr_traj_mean_straight_x, reconstr_traj_mean_straight_y = np.dot(psi.T, x_weights_straight[:, None]), np.dot(psi.T, y_weights_straight[:, None])

    err_right_x = math.sqrt(np.mean((reconstr_traj_mean_right_x - means_right[0]) ** 2))
    err_right_y = math.sqrt(np.mean((reconstr_traj_mean_right_y - means_right[1]) ** 2))
    err_straight_x = math.sqrt(np.mean((reconstr_traj_mean_straight_x - means_straight[0]) ** 2))
    err_straight_y = math.sqrt(np.mean((reconstr_traj_mean_straight_y - means_straight[1]) ** 2))
    err_left_x = math.sqrt(np.mean((reconstr_traj_mean_left_x - means_left[0]) ** 2))
    err_left_y = math.sqrt(np.mean((reconstr_traj_mean_left_y - means_left[1]) ** 2))

    err_right_dist = distance.euclidean(err_right_x, err_right_y)
    err_straight_dist = distance.euclidean(err_straight_x, err_straight_y)
    err_left_dist = distance.euclidean(err_left_x, err_left_y)

    return err_right_dist, err_straight_dist, err_left_dist

####################################################
################ Call For Functions ################
####################################################

# err_right = []
# err_straight = []
# err_left = []
# N = []
# for n in range(2, 10): # 0 = right,  1 = straight, 2 = left
#     result_set_A = err_on_basis_mean_traj_err1(N=n, time_steps=100)
#     err_right.append(result_set_A[0])
#     err_straight.append(result_set_A[1])
#     err_left.append(result_set_A[2])
#     N.append(n)
#
# print("N: ", N)
# print ("err [right, straight, left]: ", err_right, err_straight, err_left)
#
# fig00, axes = plt.subplots(3,1, figsize=(10,15))
#
# axes[0].set_title('Error of going Right')
# axes[1].set_title('Error of going Straight')
# axes[2].set_title('Error of for moving Right, Straight and Left')
#
#
# plt.xlabel('# of basis functions')
# axes[2].set_ylabel('Reconstructed trajectory error')
# axes[0].plot(N, err_right, 'bo')
# axes[1].plot(N, err_straight, 'mo')
# axes[2].plot(N, err_right, 'ro')
#
# plt.legend(loc='upper right')
# plt.show()
#
# # exit(1)


time_steps = 100
N=6 # Number of basis functions

###### Prepare Matrix for Basis Functions ######

z = np.linspace(-0, 1, time_steps) # starting at 0, ending at 1
a= z[:,None]
psi = define_basis_functions(z, N)

###### Labels for Graphs ######

labels = {
        'right': 'Probabilistic Prediction Model info. for going to Right, Straight, Left',
        'straight': 'Probabilistic Prediction Model info. for going  Straight',
        'left': 'Probabilistic Prediction Model info. for going to Left',
        'black': 'Recreated Traj.',
        'original': 'Testing Traj.',
        'mean': 'Mean Traj. from Probabilistic Prediction Model info.',
        'cov': 'Variance',
        'rec_mean': 'Recreated Traj. by means',
        'reconstructed': 'Reconstructed Traj. by means',
        }

###### Calculate weights ######

dataset = comb_dataset(time_steps)[:3] # get dataset
print ("# of trajectory in All Data Set: ", len(dataset[0])+len(dataset[1])+len(dataset[2]))

means_right = np.mean(dataset[0], axis=0)
means_straight = np.mean(dataset[1], axis=0)
means_left = np.mean(dataset[2], axis=0)
# The standard deviation is the square root of the variance
std_right = np.std(dataset[0], axis=0)
std_straight = np.std(dataset[1], axis=0)
std_left = np.std(dataset[2], axis=0)

# calculate weight for each direction
weights_right, weights_left, weights_straight = learn_weights(dataset[0], psi), learn_weights(dataset[1], psi), learn_weights(dataset[2], psi)

# take mean on weights
x_weights_right, y_weights_right  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
x_weights_left, y_weights_left  = np.mean(weights_left[:,0], axis=0), np.mean(weights_left[:,1], axis=0)
x_weights_straight, y_weights_straight  = np.mean(weights_straight[:,0], axis=0), np.mean(weights_straight[:,1], axis=0)

# Calculation of reconstructed trajectory based on means
reconstr_traj_mean_right_x, reconstr_traj_mean_right_y = np.dot(psi.T, x_weights_right[:,None]),  np.dot(psi.T, y_weights_right[:,None])
reconstr_traj_mean_left_x, reconstr_traj_mean_left_y = np.dot(psi.T, x_weights_left[:,None]), np.dot(psi.T, y_weights_left[:,None])
reconstr_traj_mean_straight_x, reconstr_traj_mean_straight_y = np.dot(psi.T, x_weights_straight[:,None]), np.dot(psi.T, y_weights_straight[:,None])

# plotting basis functions
fig0, axes = plt.subplots(4,1, figsize=(20,20))
axes[0].set_title('Time Steps')
axes[1].set_ylabel('x value of trajectories')
axes[2].set_ylabel('y value of trajectories')
axes[3].set_ylabel('Full Trajectory')

axes[0].plot(psi.T)
axes[0].set_ylim([0, 1])

# for i in range(len(dataset[0])):
#     axes[1].plot(dataset[0][i][:, 0], color="grey", alpha=0.1)
#     axes[2].plot(dataset[0][i][:, 1], color="grey", alpha=0.1)
#     axes[3].plot(dataset[0][i][:, 0], dataset[0][i][:, 1], color="grey", alpha=0.1, label=labels['right'])
#     # shadedErrorBar(i, x_weights_right, 2 * np.sqrt(x_weights_right), color='blue', transparent=1., ax=axes[i])
#     labels['right'] = "_nolegend_"
#
# for i in range(len(dataset[1])):
#     axes[1].plot(dataset[1][i][:, 0], color="grey", alpha=0.1)
#     axes[2].plot(dataset[1][i][:, 1], color="grey", alpha=0.1)
#     # axes[3].plot(dataset[1][i][:, 0], dataset[1][i][:, 1], color="grey", alpha=0.1, label=labels['straight'])
#     labels['straight'] = "_nolegend_"
#
# for i in range(len(dataset[2])):
#     axes[1].plot(dataset[2][i][:, 0], color="grey", alpha=0.1)
#     axes[2].plot(dataset[2][i][:, 1], color="grey", alpha=0.1)
#     # axes[3].plot(dataset[2][i][:, 0], dataset[2][i][:, 1], color="grey", alpha=0.1, label=labels['left'])
#     labels['left'] = "_nolegend_"

######  ploting mean and variance, baced on test data ######
axes[1].plot(means_right[:, 0], color="black", alpha = 1.0)
axes[2].plot(means_right[:, 1], color="black", alpha = 1.0)
axes[3].plot(means_right[:, 0], means_right[:, 1], color="black", alpha = 1.0, label=labels['mean'])
# axes[3].plot(reconstr_traj_mean_right_x[:], reconstr_traj_mean_right_y[:], color="red", alpha = 1.0, label=labels['mean'])

axes[1].plot(means_straight[:, 0], color="black", alpha = 1.0)
axes[2].plot(means_straight[:, 1], color="black", alpha = 1.0)
axes[3].plot(means_straight[:, 0], means_straight[:, 1], color="black", alpha = 1.0)
# axes[3].plot(reconstr_traj_mean_straight_x[:], reconstr_traj_mean_straight_y[:], color="red", alpha = 1.0)

axes[1].plot(means_left[:, 0], color="black", alpha = 1.0)
axes[2].plot(means_left[:, 1], color="black", alpha = 1.0)
axes[3].plot(means_left[:, 0], means_left[:, 1], color="black", alpha = 1.0)
# axes[3].plot(reconstr_traj_mean_left_x[:], reconstr_traj_mean_left_y[:], color="red", alpha = 1.0)

circles_right = []
circles_right_x = []
circles_right_y = []
circles_straight = []
circles_straight_x = []
circles_straight_y = []
circles_left = []
circles_left_x = []
circles_left_y = []
for i in range(time_steps):
    circle_right = plt.Circle((means_right[i][0], means_right[i][1]), radius = 2 * np.linalg.norm(std_right[i]))
    circle_straight = plt.Circle((means_straight[i][0], means_straight[i][1]), radius = 2 * np.linalg.norm(std_straight[i]))
    circle_left = plt.Circle((means_left[i][0], means_left[i][1]), radius = 2 * np.linalg.norm(std_left[i]))
    # 1*std --> 68.27%, 2*std --> 95.45%, 3*std --> 99.73%.
    circle_right_x = plt.Circle((i, means_right[i][0]), radius = 2 * np.linalg.norm(std_right[i]))
    circle_right_y = plt.Circle((i, means_right[i][1]), radius = 2 * np.linalg.norm(std_right[i]))
    circle_straight_x = plt.Circle((i, means_straight[i][0]), radius = 2 * np.linalg.norm(std_straight[i]))
    circle_straight_y = plt.Circle((i, means_straight[i][1]), radius = 2 * np.linalg.norm(std_straight[i]))
    circle_left_x = plt.Circle((i, means_left[i][0]), radius = 2 * np.linalg.norm(std_left[i]))
    circle_left_y = plt.Circle((i, means_left[i][1]), radius = 2 * np.linalg.norm(std_left[i]))


    circles_right.append(circle_right)
    circles_right_x.append(circle_right_x)
    circles_right_y.append(circle_right_y)
    circles_straight.append(circle_straight)
    circles_straight_x.append(circle_straight_x)
    circles_straight_y.append(circle_straight_y)
    circles_left.append(circle_left)
    circles_left_x.append(circle_left_x)
    circles_left_y.append(circle_left_y)

p = PatchCollection(circles_right, alpha=0.1, color="grey")
p_x = PatchCollection(circles_right_x, alpha=1, color="grey")
p_y = PatchCollection(circles_right_y, alpha=1, color="grey")
p1 = PatchCollection(circles_straight, alpha=0.1, color="grey")
p1_x = PatchCollection(circles_straight_x, alpha=1, color="grey")
p1_y = PatchCollection(circles_straight_y, alpha=1, color="grey")
p2 = PatchCollection(circles_left, alpha=0.1, color="grey")
p2_x = PatchCollection(circles_left_x, alpha=1, color="grey")
p2_y = PatchCollection(circles_left_y, alpha=1, color="grey", label=labels['cov'])

axes[1].add_collection(p_x)
axes[1].add_collection(p1_x)
axes[1].add_collection(p2_x)
axes[2].add_collection(p_y)
axes[2].add_collection(p1_y)
axes[2].add_collection(p2_y)
axes[3].add_collection(p)
axes[3].add_collection(p1)
axes[3].add_collection(p2)

fig, ax = plt.subplots()
plt.figure()
plt.xlabel('x [m]')
plt.ylabel('y [m]')

for i in range(len(dataset[0])):
    plt.plot(dataset[0][i][:, 0], dataset[0][i][:, 1], color="grey", alpha=0.1, label=labels['right'])
    # shadedErrorBar(i, x_weights_right, 2 * np.sqrt(x_weights_right), color='blue', transparent=1., ax=axes[i])
    labels['right'] = "_nolegend_"
for i in range(len(dataset[1])):
    plt.plot(dataset[1][i][:, 0], dataset[1][i][:, 1], color="grey", alpha=0.1) #, label=labels['straight'])
    labels['straight'] = "_nolegend_"
for i in range(len(dataset[2])):
    plt.plot(dataset[2][i][:, 0], dataset[2][i][:, 1], color="grey", alpha=0.1) #, label=labels['left'])
    labels['left'] = "_nolegend_"

plt.plot(means_right[:, 0], means_right[:, 1], color="black", alpha = 1.0, label=labels['mean'])
plt.plot(reconstr_traj_mean_right_x[:], reconstr_traj_mean_right_y[:], "r-", alpha = 1.0, label=labels['reconstructed'])
plt.plot(means_straight[:, 0], means_straight[:, 1], color="black", alpha = 1.0)
plt.plot(reconstr_traj_mean_straight_x[:], reconstr_traj_mean_straight_y[:], "r-", alpha = 1.0)
plt.plot(means_left[:, 0], means_left[:, 1], color="black", alpha = 1.0)
plt.plot(reconstr_traj_mean_left_x[:], reconstr_traj_mean_left_y[:], "r-", alpha = 1.0)








plt.legend(loc='upper left')
plt.show()

##### plot reconstructed trajectory side by side to randomly selected test trajectory #####

# len_trajectory = [len(dataset[0]), len(dataset[1]), len(dataset[2])]
# # NOT TO CHOSE 1
# selected_direction = 1 # 0 = right,  1 = straight, 2 = left
# random_trajectory_count = 1 # randomly selets chosen number of trajectories
#
#
#
# for i in range(0,random_trajectory_count):
#     random_trajectory_index = random.randint(0,len_trajectory[selected_direction])
#
#     traj_x = dataset[selected_direction][random_trajectory_index][:, 0]
#     traj_y = dataset[selected_direction][random_trajectory_index][:, 1]
#
#     weights = learn_weights(np.array(dataset[selected_direction]), psi) # taking weights from all data set per direction
#     w_x = weights[i,0]
#     w_y = weights[i,1]
#
#     reconstr_traj_x = np.dot(psi.T, w_x[:,None])
#     reconstr_traj_y = np.dot(psi.T, w_y[:,None])
#
#
#     err_x = math.sqrt(np.mean((reconstr_traj_x - traj_x) ** 2))
#     err_y = math.sqrt(np.mean((reconstr_traj_y - traj_y) ** 2))
#     # err_x = math.sqrt(((reconstr_traj_x - traj_x) ** 2))
#     # err_y = math.sqrt(((reconstr_traj_y - traj_y) ** 2))
#     print ("Error [x, y]: ", err_x, err_y)
#
#     fig1, axes = plt.subplots(1, 3)
#
#     if selected_direction == 0:
#         axes[0].plot(reconstr_traj_mean_right_x, color="blue")
#         axes[1].plot(reconstr_traj_mean_right_y, color="blue")
#         axes[2].plot(reconstr_traj_mean_right_x, reconstr_traj_mean_right_y, color="blue", label=labels['rec_mean'])
#     elif selected_direction == 2:
#         axes[0].plot(reconstr_traj_mean_straight_x, color="blue")
#         axes[1].plot(reconstr_traj_mean_straight_y, color="blue")
#         axes[2].plot(reconstr_traj_mean_straight_x, reconstr_traj_mean_straight_y, color="blue", label=labels['rec_mean'])
#     elif selected_direction == 1: # NOT TO CHOSE 1
#         axes[0].plot(reconstr_traj_mean_left_x, color="blue")
#         axes[1].plot(reconstr_traj_mean_left_y, color="blue")
#         axes[2].plot(reconstr_traj_mean_left_x, reconstr_traj_mean_left_y, color="blue", label=labels['rec_mean'])
#
#
#     axes[0].plot(traj_x, color="red")
#     axes[0].plot(reconstr_traj_x, color="black")
#     axes[1].plot(traj_y, color="red")
#     axes[1].plot(reconstr_traj_y, color="black")
#     axes[2].plot(traj_x, traj_y, color="red", label=labels['original'])
#     axes[2].plot(reconstr_traj_x, reconstr_traj_y, color="black", label=labels['black'])
#
#     axes[0].set_title('x values of trajectories')
#     axes[1].set_title('y values of trajectories')
#     axes[2].set_title('Full trajectory')
#     plt.legend(loc='lower right')
# plt.show()
#
# exit(1)

# ##### reconstruct and plot trajectory from small data set #####
#
# len_trajectory = [len(dataset[0]), len(dataset[1]), len(dataset[2])]
#
# direction = 0  # 0 = right,  1 = straight, 2 = left
# dataset_trajectory_count = 3  # randomly selets chosen number of trajectories
#
# random_dataset_collection = []
#
# for i in range(0, dataset_trajectory_count):
#     random_trajectory_index = random.randint(0, len_trajectory[direction])
#
#     # traj_x = dataset[direction][random_trajectory_index][:, 0]
#     # traj_y = dataset[direction][random_trajectory_index][:, 1]
#     traj_xy = dataset[direction][random_trajectory_index]
#
#     random_dataset_collection.append(traj_xy)
#
#
# weights_limited_dataset = learn_weights(np.array(random_dataset_collection), psi)  # taking weights from selected data set (per direction)
# x_weights_limited_dataset, y_weights_limited_dataset = np.mean(weights_limited_dataset[:,0], axis=0), np.mean(weights_limited_dataset[:,1], axis=0)
#
# reconstr_traj_limited_dataset_x = np.dot(psi.T, x_weights_limited_dataset[:, None])
# reconstr_traj_limited_dataset_y = np.dot(psi.T, y_weights_limited_dataset[:, None])
#
# fig2,axes = plt.subplots(1,3)
# for i in range(len(random_dataset_collection)):
#     axes[0].plot(random_dataset_collection[i][:, 0], color="red", alpha=0.4)
#     axes[1].plot(random_dataset_collection[i][:, 1], color="red", alpha=0.4)
#     axes[2].plot(random_dataset_collection[i][:, 0], random_dataset_collection[i][:, 1], color="red", alpha=0.4, label=labels['original'])
#     labels['original'] = "_nolegend_"
#
# axes[0].plot(reconstr_traj_limited_dataset_x, color="black", alpha=1)
# axes[1].plot(reconstr_traj_limited_dataset_y, color="black", alpha=1)
# axes[2].plot(reconstr_traj_limited_dataset_x, reconstr_traj_limited_dataset_y, color="black", alpha=1, label=labels['black'])
#
# axes[0].set_title('x values of trajectories')
# axes[1].set_title('y values of trajectories')
# axes[2].set_title('Full trajectory')
# plt.legend(loc='lower right')
# plt.show()