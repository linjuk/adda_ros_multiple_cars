import matplotlib.pyplot as plt
from numpy import genfromtxt
import numpy as np

plt.style.use("seaborn")

files_dictionary = {
    'left': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_back_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_parking_front_5.csv',
    ],
    'right': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_back_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_parking_front_5.csv',
    ],
    'straight': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/back_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/back_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/back_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/back_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/back_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/forward_5.csv'
    ],
}

# container dictionary for ploting graphs
trajectory_csv_data = {}

# container dictionary for averaging
trajectory_csv_file_wise_data = {}

# color map for graph
color_map = {
    'left': 'green',
    'right': 'blue',
    'straight': 'magenta'
}

# plot first graph
plt.figure(1)
plt.title('Movement Clasification f = y(x)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')

# labels
plt.plot(0, 0, color='green', label='Left-Side Parking')
plt.plot(0, 0, color='blue', label='Right-Side Parking')
plt.plot(0, 0, color='magenta', label='Straight-Side Parking')

# loop over trajectories i.e. left, right, straight
for key in files_dictionary:
    trajectory_csv_file_wise_data[key] = {}
    # loop over files in each trajectory
    for index, file in enumerate(files_dictionary[key]):
        # read file
        trajectory_csv_data[key] = (genfromtxt(file, dtype=float, delimiter=",", skip_header=1))
        # aggregate data in container for averaging
        trajectory_csv_file_wise_data[key][index] = []
        trajectory_csv_file_wise_data[key][index].append(trajectory_csv_data[key])
        # segregate x and y for plotting
        x, y = trajectory_csv_data[key][:, 1], trajectory_csv_data[key][:, 2]
        p = plt.plot(x, y, color=color_map[key])

plt.legend(loc='lower right')
plt.show()

