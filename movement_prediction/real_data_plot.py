from functions import read_csv_fast, interpolate_dist
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

random_trajectory_left = read_csv_fast('trajectories/0_real_left.csv')
random_trajectory_right = read_csv_fast('trajectories/0_real_right.csv')
random_trajectory_straight = read_csv_fast('trajectories/0_real_straight.csv')

# random_trajectory_right = read_csv_fast('trajectories/0_test_right.csv')
# random_trajectory_left = read_csv_fast('trajectories/0_test_left.csv')

# random_trajectory_left = read_csv_fast('trajectories/0_test_test.csv')

# labels
plt.title('Real Car Data')
plt.xlabel('x')
plt.ylabel('y')

labels = {
        'original': 'Original trajectory',
        'interpolated': 'Original trajectory interpolated to 10 points',
        'right': 'Going to Right',
        'straight': 'Going Straight',
        'left': 'Going to Left'
    }



# plt.plot(0, 0)
plt.plot(random_trajectory_left[:,0], random_trajectory_left[:,1], '--ro', label=labels['left'])
plt.plot(random_trajectory_right[:,0], random_trajectory_right[:,1], '--bo', label=labels['right'])
plt.plot(random_trajectory_straight[:,0], random_trajectory_straight[:,1], '--mo', label=labels['straight'])

# plt.plot(random_trajectory_right[:,0], random_trajectory_right[:,1], '--bo', label=labels['right'])
# plt.plot(random_trajectory_left[:,0], random_trajectory_left[:,1], '--ro', label=labels['left'])

# plt.plot(random_trajectory_left[:,0], random_trajectory_left[:,1], '--ro', label=labels['left'])


plt.legend(loc='lower right')
plt.show()


