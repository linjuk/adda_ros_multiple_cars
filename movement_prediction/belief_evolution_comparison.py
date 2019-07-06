from functions import read_csv_fast, interpolate_dist, calculate_covariance_for_class, probability, belief_update, simple_probability, simple_belief_update, recognize_map
from tabulate import tabulate
import numpy as np
import glob
import math
import random
import sys

import matplotlib.pyplot as plt
plt.style.use("seaborn")

"""
Step 0:
- Recognize map
"""
# map_type = recognize_map('maps/testmap6_0_.png')
# map_type = 't-intersection'
map_type = 'x-intersection'

"""
Step 1:
- Define number of point for interpolation purposes throughout this program
- Load all trajectory files in file dictionary
- Select a test trajectory and interpolate it
"""

# set number of points
Number_Of_Points=100

# fill files directionary with file paths of all csv files
files_dictionary = {

    'right': glob.glob('trajectories/right_*.csv'),       # return all files starting with right in the folder
    'straight': glob.glob('trajectories/straight_*.csv'), # return all files starting with straight in the folder
    'left': glob.glob('trajectories/left_*.csv'),  # return all files starting with left_ in the folder
}

random_trajectory = read_csv_fast('trajectories/straight_101.csv')
interpolated_random_trajectory = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], Number_Of_Points)

random_trajectory = np.asarray(random_trajectory)
[xn, yn] = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], Number_Of_Points)
random_trajectory = np.vstack((xn, yn)).T

"""
Step 2: Accumulate and plot all trajectories
"""
trajectory_csv_data = {}            # container dictionary for ploting graphs
trajectory_csv_file_wise_data = {}  # container dictionary for averaging
all_points_from_files = []          # big array container for all points from all trajectories

# loop over trajectories i.e. left, right, straight
for key in files_dictionary:
    trajectory_csv_file_wise_data[key] = {}

    for index, file in enumerate(files_dictionary[key]):                           # loop over files in each trajectory
        file_raw_data = read_csv_fast(file)
        all_points_from_files.append(file_raw_data)
        trajectory_csv_data[key] = (file_raw_data)                                 # read file
        trajectory_csv_file_wise_data[key][index] = []                             # aggregate data in container for averaging
        trajectory_csv_file_wise_data[key][index].append(trajectory_csv_data[key])
        x, y = trajectory_csv_data[key][:, 1], trajectory_csv_data[key][:, 2]      # segregate x and y for plotting
        # p = plt.plot(x, y, color=color_map[key], alpha=0.3)

"""
Step 3: Interpolate the accumulated trajectories in the previous step to NUMBER_POINTS defined in STEP 1
"""
all_points_from_files = np.asarray(all_points_from_files)
count_right_files = len(files_dictionary['right'])
count_straight_files = len(files_dictionary['straight'])
count_left_files = len(files_dictionary['left'])


all_rights = []
for i in range(count_straight_files, count_straight_files + count_right_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
    points = np.vstack((xn, yn)).T
    all_rights.append(points)

all_straight = []
for i in range(count_straight_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
    points = np.vstack((xn, yn)).T
    all_straight.append(points)

all_lefts = []
for i in range(count_straight_files + count_right_files,
               count_straight_files + count_right_files + count_left_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
    points = np.vstack((xn, yn)).T
    all_lefts.append(points)

all_rights = np.asarray(all_rights)
all_straight = np.asarray(all_straight)
all_lefts = np.asarray(all_lefts)

"""
Step 4: Calculate and plot mean for each class i.e. all trajectories for that class
"""
means_right = np.mean(all_rights, axis=0)
means_straight = np.mean(all_straight, axis=0)
means_left = np.mean(all_lefts, axis=0)

"""
Step 5: Calculate covariance for trajectories in each class
"""
covariance_right = calculate_covariance_for_class(all_rights, Number_Of_Points)
covariance_straight = calculate_covariance_for_class(all_straight, Number_Of_Points)
covariance_left = calculate_covariance_for_class(all_lefts, Number_Of_Points)

"""
Step 6a: Comparison of belief updates between 100 and 10 steps (With Scaling)
"""
T = 100 # change the same in 26
t = 10
# change power in 137 line

Pobs = []
for i in range(1, T):
    Pobs.append([ simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                  simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i]),
                  simple_probability(random_trajectory[i], means_left[i], covariance_left[i])])

Pobs = np.array(Pobs)

# initialize prior based on recognized map type
if map_type == "t-intersection":
    prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    b100 = np.array([prior_left, prior_right, 0.0])
    b10 = np.array([prior_left, prior_right, 0.0])
else:
    b100 = np.array([0.333, 0.333, 0.333])
    b10 = np.array([0.333, 0.333, 0.333])


b100_all = []
b10_all = []


for i in range (0, T):

    if i == 0:
        print("10: ", b10, "100: ", b100)
        b10_all.append(b10)
        b100_all.append(b100)

    else:
        P = np.power(Pobs[i-1, :], 0.1)
        b100 = b100 * P
        b100 = b100 / np.sum(b100)
        b100_all.append(b100)
        # print("For 100: timestep {},  P for 100 {}, belief input for b100 {}".format(i, P, b100))

        if i % t == 0:
            P = Pobs[i-1, :]
            # print("For 10: timestep {}, P for 10 {}, belief input for b10 {}".format(i, P, b10))
            b10 = b10 * P
            b10 = b10 / np.sum(b10)
            b10_all.append(b10)
            print("10: ", b10, "100: ", b100)


"""
Step 6b: Comparison of belief updates between 100 and 10 steps (Without Scaling)
"""

Pobs = []
for i in range(1, T):
    Pobs.append([simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                 simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i]),
                 simple_probability(random_trajectory[i], means_left[i], covariance_left[i])])

Pobs = np.array(Pobs)

# initialize prior based on recognized map type
if map_type == "t-intersection":
    prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    b100 = np.array([prior_left, prior_right, 0.0])
    b10 = np.array([prior_left, prior_right, 0.0])
else:
    b100 = np.array([0.3, 0.3, 0.3])
    b10 = np.array([0.3, 0.3, 0.3])

b100_all_ws = []
b10_all_ws = []


for i in range (0, T):

    if i == 0:
        print("10: ", b10, "100: ", b100)
        b10_all_ws.append(b10)
        b100_all_ws.append(b100)

    else:
        P = Pobs[i-1, :]
        b100 = b100 * P
        b100 = b100 / np.sum(b100)
        b100_all_ws.append(b100)
        # print("For 100: timestep {},  P for 100 {}, belief input for b100 {}".format(i, P, b100))

        if i % t == 0:
            P = Pobs[i-1, :]
            # print("For 10: timestep {}, P for 10 {}, belief input for b10 {}".format(i, P, b10))
            b10 = b10 * P
            b10 = b10 / np.sum(b10)
            b10_all_ws.append(b10)
            # print("10: ", b10, "100: ", b100)

"""
Step 7a: Plotting (with scaling)
"""

# color map for graph
color_map = {
    'right': 'blue',
    'straight': 'magenta',
    'left': 'green'
}

# plot graph
fig, ax = plt.subplots()
plt.figure(1)
plt.title('Belief Updates over Time (with scaling)')
plt.xlabel('Time Steps')
plt.ylabel('Belief over Class')

labels = {

    'right100': 'Belief for going right_100 steps',
    'straight100': 'Belief for going straight_100 steps',
    'left100': 'Belief for going left_100 steps',
    'right10': 'Belief for going right_10 steps',
    'straight10': 'Belief for going straight_10 steps',
    'left10': 'Belief for going left_10 steps'
}

b10_counter = 0
for i in range(0, T):

    # plt.plot(i, b100_all[i][0], marker=".", color="blue", label=labels['right100'], alpha=0.4)
    # plt.plot(i, b100_all[i][1], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)
    # plt.plot(i, b100_all[i][2], marker=".", color="green", label=labels['left100'], alpha=0.4)

    if i % t == 0:
        plt.plot(i, b100_all[i][0], marker=".", color="blue", label=labels['right100'], alpha=0.4)
        plt.plot(i, b100_all[i][1], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)
        plt.plot(i, b100_all[i][2], marker=".", color="green", label=labels['left100'], alpha=0.4)

        plt.plot(i, b10_all[b10_counter][0], marker="D", color="blue", label=labels['right10'], alpha=0.4)
        plt.plot(i, b10_all[b10_counter][1], marker="D", color="magenta", label=labels['straight10'], alpha=0.4)
        plt.plot(i, b10_all[b10_counter][2], marker="D", color="green", label=labels['left10'], alpha=0.4)
        b10_counter+=1

    # ignore legend after first print
    for key in labels:
        labels[key] = "_nolegend_"

plt.legend(loc='center right')



# plt.show()

"""
Step 7b: Plotting (without scaling)
"""

# color map for graph
color_map = {
    'left': 'green',
    'right': 'blue',
    'straight': 'magenta'
}

# plt.legend(loc='lower right')
# plot graph
fig, ax = plt.subplots()
plt.figure(2)
plt.title('Belief Updates over Time (without scaling)')
plt.xlabel('Time Steps')
plt.ylabel('Belief over Class')

labels_ws = {
    'right100': 'Belief for going right_100 steps',
    'straight100': 'Belief for going straight_100 steps',
    'left100': 'Belief for going left_100 steps',
    'right10': 'Belief for going right_10 steps',
    'straight10': 'Belief for going straight_10 steps',
    'left10': 'Belief for going left_10 steps'
}

b10_counter = 0
for i in range(0, T):

    plt.plot(i, b100_all_ws[i][0], marker=".", color="blue", label=labels_ws['right100'], alpha=0.4)
    plt.plot(i, b100_all_ws[i][1], marker=".", color="magenta", label=labels_ws['straight100'], alpha=0.4)
    plt.plot(i, b100_all_ws[i][2], marker=".", color="green", label=labels_ws['left100'], alpha=0.4)

    if i % t == 0:
        plt.plot(i, b10_all_ws[b10_counter][0], marker="D", color="blue", label=labels_ws['right10'], alpha=0.4)
        plt.plot(i, b10_all_ws[b10_counter][1], marker="D", color="magenta", label=labels_ws['straight10'], alpha=0.4)
        plt.plot(i, b10_all_ws[b10_counter][2], marker="D", color="green", label=labels_ws['left10'], alpha=0.4)
        b10_counter+=1

    # ignore legend after first print
    for key in labels_ws:
        labels_ws[key] = "_nolegend_"

plt.legend(loc='center right')
plt.show()
