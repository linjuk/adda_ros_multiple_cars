import numpy as np
import matplotlib.pylab as plt
import padasip as pa
from astropy.modeling import models, fitting
from functions import read_csv_fast, interpolate_dist

# testing trajectories
test_trajectory_left = read_csv_fast('trajectories/test_left_100.csv')
test_trajectory_right = read_csv_fast('trajectories/test_right_100.csv')
test_trajectory_straight = read_csv_fast('trajectories/test_straight_100.csv')

# creation of basis function
N = 100

######## 1st Radial Basis Function ########

x1 = np.random.normal(0, 0.1, (N, 2))
y1 = np.exp(-(x1 - 0)**2 / 0.2**2)
print(x1)
# noise = 0.00001
interpolated_right = interpolate_dist(test_trajectory_right[:, 1], test_trajectory_right[:, 2], N)

x_test_tr_right = interpolated_right[0] # x coordinates of test trajectory for going to the right
y_test_tr_right = interpolated_right[1] # x coordinates of test trajectory for going to the right

# weights calculation -> least mean squares
f = pa.filters.FilterLMS(n=2, mu=1, w="zeros")
x_right, e_x_right, w_x_right = f.run(x_test_tr_right, x1)
print("weights_right_x: ", len(w_x_right), w_x_right)
y_right, e_y_right, w_y_right = f.run(y_test_tr_right, x1)
print("weights_right_y: ", len(w_y_right), w_y_right)

# # plot results
plt.figure(figsize=(8,5))
plt.plot(x1, y1, label='$\phi_1$')
# plt.axis([0, 1, 0, 1])
plt.legend(loc='center left')
plt.show()

plt.plot(x1)
plt.show()