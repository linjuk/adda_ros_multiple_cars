import numpy as np
import matplotlib.pyplot as plt
import random
from functions import comb_dataset, read_csv_fast, interpolate_dist

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


####################################################
################ Call For Functions ################
####################################################

time_steps = 100
N=5 # Number of basis functions

###### Prepare Matrix for Basis Functions ######

z = np.linspace(-0, 1, time_steps) # starting at 0, ending at 1
a= z[:,None]
psi = define_basis_functions(z, N)

###### Labels for Graphs ######

labels = {
        'right': 'Going to Right',
        'straight': 'Going Straight',
        'left': 'Going to Left',
        'black': 'Recreated Traj.',
        'original': 'Testing Traj.'
    }

###### Calculate weights ######

# get dataset
dataset = comb_dataset(time_steps)[:3]
print ("# of trajectory in All Data Set: ", len(dataset[0])+len(dataset[1])+len(dataset[2]))

# calculate weight for each direction
weights_right, weights_left, weights_straight = learn_weights(dataset[0], psi), learn_weights(dataset[1], psi), learn_weights(dataset[2], psi)

# take mean on weights
x_weights_right, y_weights_right  = np.mean(weights_right[:,0], axis=0), np.mean(weights_right[:,1], axis=0)
x_weights_left, y_weights_left  = np.mean(weights_left[:,0], axis=0), np.mean(weights_left[:,1], axis=0)
x_weights_straight, y_weights_straight  = np.mean(weights_straight[:,0], axis=0), np.mean(weights_straight[:,1], axis=0)

# plotting basis functions
fig0, axes = plt.subplots(4,1, figsize=(20,20))

axes[0].set_title('Time Steps')
axes[1].set_ylabel('x value of trajectories')
axes[2].set_ylabel('y value of trajectories')
axes[3].set_ylabel('Full Trajectory')

axes[0].plot(psi.T)
axes[0].set_ylim([0, 1])

for i in range(len(dataset[0])):
    axes[1].plot(dataset[0][i][:, 0], color="grey", alpha=0.1)
    axes[2].plot(dataset[0][i][:, 1], color="grey", alpha=0.1)
    axes[3].plot(dataset[0][i][:, 0], dataset[0][i][:, 1], color="grey", alpha=0.1, label=labels['right'])

    # shadedErrorBar(i, x_weights_right, 2 * np.sqrt(x_weights_right), color='blue', transparent=1., ax=axes[i])

    labels['right'] = "_nolegend_"

for i in range(len(dataset[1])):
    axes[1].plot(dataset[1][i][:, 0], color="grey", alpha=0.1)
    axes[2].plot(dataset[1][i][:, 1], color="grey", alpha=0.1)
    axes[3].plot(dataset[1][i][:, 0], dataset[1][i][:, 1], color="grey", alpha=0.1, label=labels['straight'])
    labels['straight'] = "_nolegend_"

for i in range(len(dataset[2])):
    axes[1].plot(dataset[2][i][:, 0], color="grey", alpha=0.1)
    axes[2].plot(dataset[2][i][:, 1], color="grey", alpha=0.1)
    axes[3].plot(dataset[2][i][:, 0], dataset[2][i][:, 1], color="grey", alpha=0.1, label=labels['left'])
    labels['left'] = "_nolegend_"

reconstr_traj_right_x, reconstr_traj_right_y = np.dot(psi.T, x_weights_right[:,None]),  np.dot(psi.T, y_weights_right[:,None])
reconstr_traj_left_x, reconstr_traj_left_y = np.dot(psi.T, x_weights_left[:,None]), np.dot(psi.T, y_weights_left[:,None])
reconstr_traj_straight_x, reconstr_traj_straight_y = np.dot(psi.T, x_weights_straight[:,None]), np.dot(psi.T, y_weights_straight[:,None])

axes[1].plot(reconstr_traj_right_x, color="black", alpha=0.5)
axes[2].plot(reconstr_traj_right_y, color="black", alpha=0.5)
axes[3].plot(reconstr_traj_right_x, reconstr_traj_right_y, color="black", alpha=1, label=labels['black'])
axes[1].plot(reconstr_traj_left_x, color="black", alpha=0.5)
axes[2].plot(reconstr_traj_left_y, color="black", alpha=0.5)
axes[3].plot(reconstr_traj_left_x, reconstr_traj_left_y, color="black", alpha=1)
axes[1].plot(reconstr_traj_straight_x, color="black", alpha=0.5)
axes[2].plot(reconstr_traj_straight_y, color="black", alpha=0.5)
axes[3].plot(reconstr_traj_straight_x, reconstr_traj_straight_y, color="black", alpha=0.5)

plt.legend(loc='upper left')
plt.show()


##### plot reconstructed trajectory side by side to randomly selected test trajectory #####

len_trajectory = [len(dataset[0]), len(dataset[1]), len(dataset[2])]

selected_direction = 2 # 0 = right,  1 = straight, 2 = left
random_trajectory_count = 1 # randomly selets chosen number of trajectories

for i in range(0,random_trajectory_count):
    random_trajectory_index = random.randint(0,len_trajectory[selected_direction])

    traj_x = dataset[selected_direction][random_trajectory_index][:, 0]
    traj_y = dataset[selected_direction][random_trajectory_index][:, 1]

    weights = learn_weights(np.array(dataset[selected_direction]), psi) # taking weights from all data set per direction
    w_x = weights[i,0]
    w_y = weights[i,1]



    reconstr_traj_x = np.dot(psi.T, w_x[:,None])
    reconstr_traj_y = np.dot(psi.T, w_y[:,None])

    # err_x = abs(np.mean((reconstr_traj_x-traj_x) / traj_x))
    # err_y = abs(np.mean((reconstr_traj_y-traj_y) / traj_y))
    # print("Error [x, y]: ", err_x, err_y)

    # err_x1 = abs(np.mean((traj_x-reconstr_traj_x) ** 2))
    # err_y1 = abs(np.mean((traj_y-reconstr_traj_y) ** 2))
    # print("Error [x1, y1]: ", err_x1, err_y1)

    # print (reconstr_traj_x, traj_x)

    fig1,axes = plt.subplots(1,3)
    axes[0].plot(traj_x, color="red")
    axes[0].plot(reconstr_traj_x, color="black")
    axes[1].plot(traj_y, color="red")
    axes[1].plot(reconstr_traj_y, color="black")
    axes[2].plot(traj_x, traj_y, color="red", label=labels['original'])
    axes[2].plot(reconstr_traj_x, reconstr_traj_y, color="black", label=labels['black'])

    axes[0].set_title('x values of trajectories')
    axes[1].set_title('y values of trajectories')
    axes[2].set_title('Full trajectory')
    plt.legend(loc='lower right')
    plt.show()



exit(1)

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

