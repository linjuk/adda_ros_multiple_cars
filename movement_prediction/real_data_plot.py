from functions import read_csv_fast, interpolate_dist
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

random_trajectory_left = read_csv_fast('trajectories/0_RealData_left.csv')
random_trajectory_straight = read_csv_fast('trajectories/0_RealData_straight.csv')
random_trajectory_right = read_csv_fast('trajectories/0_RealData_right.csv')

# labels
plt.title('Real Car Data')
plt.xlabel('x, m')
plt.ylabel('y, m')

labels = {
        'right': 'Going to Right',
        'straight': 'Going Straight',
        'left': 'Going to Left'
    }

# plt.plot(0, 0)
plt.plot(random_trajectory_left[:,0], random_trajectory_left[:,1], 'green', label=labels['left'])
plt.plot(random_trajectory_left[:,2], random_trajectory_left[:,3], 'green')
plt.plot(random_trajectory_left[:,4], random_trajectory_left[:,5], 'green')
plt.plot(random_trajectory_left[:,6], random_trajectory_left[:,7], 'green')
plt.plot(random_trajectory_left[:,8], random_trajectory_left[:,9], 'green')

plt.plot(random_trajectory_straight[:,0], random_trajectory_straight[:,1], 'magenta', label=labels['straight'])
plt.plot(random_trajectory_straight[:,2], random_trajectory_straight[:,3], 'magenta')
plt.plot(random_trajectory_straight[:,4], random_trajectory_straight[:,5], 'magenta')
plt.plot(random_trajectory_straight[:,6], random_trajectory_straight[:,7], 'magenta')
plt.plot(random_trajectory_straight[:,8], random_trajectory_straight[:,9], 'magenta')

plt.plot(random_trajectory_right[:,0], random_trajectory_right[:,1], 'blue', label=labels['right'])
plt.plot(random_trajectory_right[:,2], random_trajectory_right[:,3], 'blue')
plt.plot(random_trajectory_right[:,4], random_trajectory_right[:,5], 'blue')
plt.plot(random_trajectory_right[:,6], random_trajectory_right[:,7], 'blue')
plt.plot(random_trajectory_right[:,8], random_trajectory_right[:,9], 'blue')

plt.legend(loc='lower right')
plt.show()