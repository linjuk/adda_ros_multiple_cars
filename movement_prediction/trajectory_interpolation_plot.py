from functions import read_csv_fast, interpolate_dist
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

random_trajectory_right = read_csv_fast('trajectories/right_03.csv')
interpolated_trajectory_right = interpolate_dist(random_trajectory_right[:,1], random_trajectory_right[:, 2], 10)

random_trajectory_straight = read_csv_fast('trajectories/straight_03.csv')
interpolated_trajectory_straight = interpolate_dist(random_trajectory_straight[:,1], random_trajectory_straight[:, 2], 10)

random_trajectory_left = read_csv_fast('trajectories/left_03.csv')
interpolated_trajectory_left = interpolate_dist(random_trajectory_left[:,1], random_trajectory_left[:, 2], 10)

# labels
plt.title('Trajectries interpolation from original numbet of time steps to 10 time steps')
plt.xlabel('x')
plt.ylabel('y')

labels = {
        'original': 'Original trajectory',
        'interpolated': 'Original trajectory interpolated to 10 points'
    }



plt.plot(0, 0)
plt.plot(random_trajectory_right[:,1], random_trajectory_right[:,2], '--ro', label=labels['original'])
plt.plot(interpolated_trajectory_right[0], interpolated_trajectory_right[1], '--bo', label=labels['interpolated'])

plt.plot(random_trajectory_straight[:,1], random_trajectory_straight[:,2], '--ro')
plt.plot(interpolated_trajectory_straight[0], interpolated_trajectory_straight[1], '--bo')

plt.plot(random_trajectory_left[:,1], random_trajectory_left[:,2], '--ro')
plt.plot(interpolated_trajectory_left[0], interpolated_trajectory_left[1], '--bo')

plt.legend(loc='lower left')
plt.show()


