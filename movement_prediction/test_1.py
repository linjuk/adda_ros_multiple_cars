
import numpy as np
import matplotlib.pyplot as plt
def define_basis_functions( z_vec, N):
    """returns a NxT matrix containing the basis functions

    :param T: number of time steps of trajectories after normalization (for this application, the demonstrations were
    post-processed in order to make all of them have the same number of time steps; in posterior works, we have used the
    concept of "phase" that allows for dealing with demonstrations with different number of time steps)
    :param N:  number of basis functions
    :return: matrix of basis functions
    """

    T = 1  # ????
    h = T

    # Define generic basis function
    # psi = lambda n, t: np.exp(-0.5 * ((t + 1) - n * T / (N - 1)) ** 2 / h)
    h = 1.0 / (N * N)
    psi = lambda n, t: np.exp(-0.5 * ((t - (float(n) / (N - 1))) ** 2 / h))

    # Define PSIs matrix
    PSI_matrix = np.zeros((N, len(z_vec)))

    for n in range(N):
        for i in range(len(z_vec)):
            z = z_vec[i]
            # print z
            PSI_matrix[n, i] = psi(n, z)

    PSI_matrix = PSI_matrix / np.sum(PSI_matrix, 0)

    return PSI_matrix


z = np.linspace(0,1,100)
a= z[:,None]
n_basis=10
psi = define_basis_functions(z,n_basis)
fig, axes = plt.subplots(2,1)
axes[0].plot(psi.T)


w= np.ones((1,n_basis))

reconstr_traj = np.dot(w,psi)
axes[1].plot(reconstr_traj.T)
plt.show()

