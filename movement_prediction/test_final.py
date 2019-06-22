import numpy as np
import matplotlib.pyplot as plt
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

####################################################
################ Call For Functions ################
####################################################

time_steps = 100
N=3 # Number of basis functions

###### Prepare Matrix for Basis Functions ######

z = np.linspace(-0, 1, time_steps) # starting at 0, ending at 1
a= z[:,None]
psi = define_basis_functions(z, N)

###### Calculate weights ######

dataset = testing_trajectory = read_csv_fast('trajectories/test_right_100.csv')
dataset = comb_dataset(time_steps)
dataset = np.concatenate((dataset[0], dataset[1], dataset[2]))
print ("# of trajectory in Data Set: ", len(dataset))
DS = dataset.shape[0]
print DS

weights = learn_weights(dataset, psi)

for i in range(0,10):#weights.shape[0]):
    wx=weights[i,0]
    wy=weights[i,1]
    traj_x=dataset[i][:, 0]
    traj_y=dataset[i][:, 1]
    reconstr_trajx = np.dot(psi.T, wx[:,None])
    reconstr_trajy = np.dot(psi.T, wy[:,None])

    #fig,ax = plt.subplots(1,2)
    #ax[0].plot(traj_x,'b')
    #ax[0].plot(reconstr_trajx,'r')
    #ax[1].plot(traj_y, 'b')
    #ax[1].plot(reconstr_trajy, 'r')
    #plt.show()



    err = np.sum((reconstr_trajx-traj_x) **2)
    print err
x_weights  = np.mean(weights[:,0], axis=0)
y_weights  = np.mean(weights[:,1], axis=0)

print("Weights for X: ", x_weights)
print("Weights for Y: ", y_weights)

###### Plotting Results ######

testing_trajectory = read_csv_fast('trajectories/test_right_100.csv')
[xn, yn] = interpolate_dist(testing_trajectory[:, 1], testing_trajectory[:, 2], time_steps)
points = np.vstack((xn, yn)).T

# plotting basis functions
fig1, axes = plt.subplots(3,1)
axes[0].plot(psi.T)
axes[0].set_ylim([0, 1])
# plotting x value of test trajectories

for i in range(len(dataset[0])):
     axes[1].plot(dataset[i][:, 0], color="blue", alpha=0.4)
     axes[2].plot(dataset[i][:, 1], color="blue", alpha=0.4)

     wx = weights[i, 0]
     wy = weights[i, 1]
     #traj_x = dataset[i][:, 0]
     #traj_y = dataset[i][:, 1]
     reconstr_trajx = np.dot(psi.T, wx[:, None])
     reconstr_trajy = np.dot(psi.T, wy[:, None])
     axes[1].plot(reconstr_trajx, color="red", alpha=0.4)
     axes[2].plot(reconstr_trajy, color="red", alpha=0.4)


# axes[1].plot(points[:, 0], color="red", alpha=0.4)
plt.xlabel('Time Steps')
plt.ylabel('x value of trajectories')
# plotting x value of reconstructed trajectory
# w= np.ones((1, N))
# reconstr_traj = np.dot(1, psi)
# axes[1].plot(reconstr_traj.T)
#x = np.zeros((1, N))
x_reconstructed = np.dot(psi.T, x_weights[:])
axes[1].plot(x_reconstructed, color="black")


y_reconstructed = np.dot(psi.T, y_weights[:])
axes[2].plot(y_reconstructed.T, color="black")

plt.show()

# plotting basis functions
#fig2, axes = plt.subplots(2,1)
#axes[0].plot(psi.T)
#axes[0].set_ylim([0, 1])
# plotting y value of test trajectories
#for i in range(len(dataset[0])):
    #axes[1].plot(dataset[i][:, 1], color="blue", alpha=0.4)
# axes[1].plot(points[:, 1], color="red", alpha=0.4)

#plt.xlabel('Time Steps')
#plt.ylabel('y value of trajectories')
## plotting y value of reconstructed trajectory
#y = np.zeros((1, N))



