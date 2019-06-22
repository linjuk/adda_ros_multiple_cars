from functions import read_csv_fast, interpolate_dist
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

random_trajectory_left_1 = read_csv_fast('trajectories/0_RealData_left_01.csv')
# random_trajectory_left_2 = read_csv_fast('trajectories/0_RealData_left_03.csv')
# random_trajectory_left_3 = read_csv_fast('trajectories/0_RealData_left_07.csv')
# # random_trajectory_left_4 = read_csv_fast('trajectories/0_RealData_left_09.csv') # weird data
# # plt.plot(random_trajectory_left_4[:,2], random_trajectory_left_4[:,3], '--ro')


random_trajectory_right_1 = read_csv_fast('trajectories/00000_real_left_1.csv')
# random_trajectory_right_2 = read_csv_fast('trajectories/0_RealData_right_05.csv')
# random_trajectory_right_3 = read_csv_fast('trajectories/0_RealData_right_09.csv')

# random_trajectory_straight = read_csv_fast('trajectories/0000000000_straight.csv')

# labels
plt.title('Real Car Data')
plt.xlabel('x')
plt.ylabel('y')

labels = {
        'right': 'Going to Right',
        'straight': 'Going Straight',
        'left': 'Going to Left'
    }

# plt.plot(0, 0)
# plt.plot(random_trajectory_left_1[:,2], random_trajectory_left_1[:,3], '--ro', label=labels['left'])
# plt.plot(random_trajectory_left_2[:,2], random_trajectory_left_2[:,3], '--ro')
# plt.plot(random_trajectory_left_3[:,2], random_trajectory_left_3[:,3], '--ro')

plt.plot(random_trajectory_right_1[:,6], random_trajectory_right_1[:,7], '--bo', label=labels['right'])
# plt.plot(random_trajectory_right_2[:,2], random_trajectory_right_2[:,3], '--bo')
# plt.plot(random_trajectory_right_3[:,2], random_trajectory_right_3[:,3], '--bo')

# plt.plot(random_trajectory_straight[:,0], random_trajectory_straight[:,1], '--mo', label=labels['straight'])


plt.legend(loc='lower right')
plt.show()


