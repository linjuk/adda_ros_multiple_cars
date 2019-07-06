import numpy as np
import matplotlib.pyplot as plt


class ProMP():

    def __init__(self ,n_basis):
        self.n_basis = n_basis

        self.ridgeregression_lambda = 1e-10

        self._prior_cov_w = 1e-3 * np.eye(self.n_basis)

        self.z = 0.0

        self.zd = 0.1

        self.data=[]

    def compute_phase_vector(self ,t ,t_end):
        return t/ t_end

    def define_basis_functions(self, z_vec, N):
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
        #psi = lambda n, t: np.exp(-0.5 * ((t + 1) - n * T / (N - 1)) ** 2 / h)
        h=1.0/(N*N)
        psi = lambda n, t: np.exp(-0.5 * ( (t  - (float(n)/(N - 1)) )** 2 / h))


        # Define PSIs matrix
        PSI_matrix = np.zeros((N, len(z_vec)))

        for n in range(N):
            for i in range(len(z_vec)):
                z = z_vec[i]
                #print z
                PSI_matrix[n, i] = psi(n, z)

        PSI_matrix = PSI_matrix / np.sum(PSI_matrix, 0)

        return PSI_matrix

    def train(self, data):
        self.data = data
        self.n_dof = data[0].shape[1]
        # data : list of M trajs, maybe with different length of timesteps , dim traj 1XT
        w = np.zeros((len(data),data[0].shape[1],self.n_basis))
        n_demo = len(data)
        T = np.zeros(len(data))
        for i in range(len(data)):
            traj = data[i]
            T[i] =len(traj)
            z = self.compute_phase_vector(np.arange(0,len(traj)).astype(float), len(traj))
            psi = self.define_basis_functions(z, self.n_basis)

            #fig1, axes1 = plt.subplots(1, 1)

            #axes1.plot(psi.T)
            #plt.show()

            w[i,:,:] = np.squeeze(self.learn_weights(np.asarray([traj]), psi,1e-3))

            # check reconstruction error
            t_reconstr = np.dot(psi.T, np.squeeze(w[i,:,:]).T)

            #fig2, axes2 = plt.subplots(1, 1)

            #axes2.plot(traj[:,0],traj[:,1],'red')
            #axes2.plot(t_reconstr[:, 0], t_reconstr[:, 1], 'blue')
            #plt.show()


            #a=0


        self.avg_T = int(np.mean(T))

        self.avg_z = self.compute_phase_vector(np.arange(0,self.avg_T).astype(float), self.avg_T)

        self.avg_Psi_matrix = self.define_basis_functions(self.avg_z, self.n_basis)

        self.w_mu = np.zeros((self.n_dof,self.n_basis))
        self.w_cov = np.zeros((self.n_dof, self.n_basis, self.n_basis))
        for i in range(self.n_dof):
            weights = np.squeeze(w[:,i,:])

            self.w_mu[i, :] = np.mean(weights,axis=0)
            self.w_cov[i,:,:] = 1./n_demo * np.dot((weights - self.w_mu[i, :]).transpose(), (weights - self.w_mu[i, :]))\
                      + self._prior_cov_w


        a=0

    def visualize_promp(self, axes,factor_x_axis = 1.0):
        time = self.avg_z
        if self.n_dof == 1:
            mean_traj = np.dot(self.avg_Psi_matrix.T, self.w_mu[0, :])
            c_traj = np.dot(self.avg_Psi_matrix.T, np.diag(self.w_cov[0, :, :]))

            self.shadedErrorBar(time*factor_x_axis, mean_traj.T, 2 * np.sqrt(c_traj), color='blue',
                                transparent=1., ax=axes)

            for d in self.data:
                b = np.asarray(d[:, 0])
                axes.plot(np.linspace(0, 1, len(b))*factor_x_axis, b, 'yellow',alpha=0.5)
                print "bla"
            return

        for dim in range(self.n_dof):
            mean_traj = np.dot(self.avg_Psi_matrix.T, self.w_mu[dim, :])
            c_traj = np.dot(self.avg_Psi_matrix.T, np.diag(self.w_cov[dim, :, :]))

            self.shadedErrorBar(time*factor_x_axis, mean_traj.T, 2 * np.sqrt(c_traj), color='blue',
                           transparent=1., ax=axes[dim])

            for d in self.data:
                b = np.asarray(d[:,dim])
                axes[dim].plot(np.linspace(0,1,len(b))*factor_x_axis, b,'yellow',alpha=0.5)
                print "bla"

    def learn_weights(self, norm_data, PSIs_matrix,LAMBDA_COEFF=1e-12):
        """
        learn regression weights for given basis functions and for given data
        :param norm_data: data to learn weights with
        :param PSIs_matrix: matrix of basis kernel functions
        :return: learned weights
        """
        # Find out the number of basis functions
        N = PSIs_matrix.shape[0]

        # Find out total number of degrees of freedom. Ex.: there are 6 degrees of
        # freedom in total if there are 2 agents and each agent has 3 degrees of
        # freedom.
        #s= norm_data[0]
        dof = norm_data[0].shape[1]

        # There is a matrix (#Dofs x #basis functions) of weights for each of the demonstrations.
        weights = np.zeros((norm_data.shape[0], dof, N))

        # fill weights matrix
        for index in range(norm_data.shape[0]):
            for i in range(dof):
                # In case some regularization is necessary
                # weights[index][i] = np.dot(np.linalg.inv(np.dot(PSIs_matrix, PSIs_matrix.T) + 10e-12 * np.eye(np.dot(PSIs_matrix, PSIs_matrix.T).shape[0])), np.dot(PSIs_matrix, norm_data[index][:,i]))
                weights[index][i] = np.dot(np.linalg.pinv(
                    np.dot(PSIs_matrix, PSIs_matrix.T) + LAMBDA_COEFF * np.identity(PSIs_matrix.shape[0])),
                                           np.dot(PSIs_matrix, norm_data[index][:, i]))

                y=norm_data[index][:, i]
                y_test = np.dot(PSIs_matrix.T,weights[index][i])
                a = PSIs_matrix
                c = weights[index][i]
                b = np.dot(
                    PSIs_matrix.T, weights[index][i])

        return weights

    def shadedErrorBar(self, x, y, errBar, color=None, transparent=0, ax=None):

        y = np.asarray(y).reshape(x.shape)
        #color = np.array(to_rgb(color))

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

    def step(self, w):
        self.z = self.z + self.zd
        psi = self.define_basis_functions([self.z], self.n_basis)
        x_t = np.dot(psi.T, w[0, :])
        y_t = np.dot(psi.T, w[1, :])

        return x_t,y_t

    def values_from_z(self,w,z):
        psi = self.define_basis_functions([z], self.n_basis)

        x = np.zeros((1,self.n_dof))
        for dim in range(self.n_dof):
            x[0,dim] = np.dot(psi.T, w[dim, :])
            # y_t = np.dot(psi.T, w[1, :])
        return x

    def traj_from_w(self,w):
        x = np.zeros((self.n_dof,self.avg_T))
        for dim in range(self.n_dof):
            x[dim,:] = np.dot(self.avg_Psi_matrix.T, w[dim, :])
        return x

    def traj_cov_from_w(self,w_cov):
        x = np.zeros((self.n_dof,self.avg_T))
        for dim in range(self.n_dof):
            x[dim,:] = np.dot(self.avg_Psi_matrix.T, np.diag(w_cov[dim, :, :]))
        return x

    def print_state(self):
        print "phase (z): " + str(self.z)
        print "zd: " + str(self.zd)



from ..utils import prepare_observation_matrix, psi_block_diag, nan_matrix
import numpy as np
import scipy

def completion_ProMPs_with_GMMs(observation, obs_start, obs_end,
obs_dof, inv_obs_noise, PSI_matrix, mu, covariance, T, nr_dof):
    """compute mean vector and covariance matrix of trajectory
conditioned on observation according to equations (
    16), (17)

    :param observation: observed points from a trajectory
    :param obs_start: start index of observation
    :param end_obs: end index of observation
    :param inv_obs_noise: variance of observation
    :param PSI_matrix: matrix of basis functions from regression
    :param mu: mean vector for Gaussian Mixture component
    :param covariance: variance matrix for Gaussian Mixture component
    :param T: number of timesteps
    :param nr_dof: total dof
    :return: mean vector and covariance matrix for the trajectory
    """

    # expand the observation to full size an reshapes it to an column verctor
    observation, indexObs = prepare_observation_matrix(observation,
obs_start, obs_end, obs_dof, T, nr_dof)

    # 1st column: index of observed values; 2nd column: observed values
    data = np.vstack((indexObs, observation[indexObs]))

    A = psi_block_diag(PSI_matrix, nr_dof)

    # take exactly the lines of A that correspond to observed values
    obsA = A[indexObs, :]
    a = inv_obs_noise

    # add some noise to avoid numerical problems
    #covariance = covariance + 1e-11* np.eye(np.size(covariance, 0),
np.size(covariance, 1))

    # Parameters of posterior in weight space
    b = scipy.linalg.inv(covariance)
    #np.testing.assert_array_almost_equal(np.dot(b, covariance),
np.diag(np.ones(270)), decimal=3)
    # calc lambda
    lam = a*(np.dot(obsA.T, obsA)) + b

    w_mu_posterior = np.dot(scipy.linalg.inv(lam), (a * np.dot(obsA.T,
data[1].reshape(data[1].shape[0], 1)) + np.dot(b, mu)))

    # Parameters of posterior in trajectory space
    u = np.dot(A, w_mu_posterior)

    variance = 1./a*np.eye(np.size(A, 0)) + np.dot(np.dot(A,
scipy.linalg.inv(lam)), A.T)

    matrix_u = np.reshape(u, (T, nr_dof), order='F')

    matrix_variance = nan_matrix((T, nr_dof))
    for i in xrange(nr_dof):
        matrix_variance[:,i] = np.diag(variance[(i * T):(i * T + T),
(i * T):(i * T + T)])

    return (matrix_u, matrix_variance)