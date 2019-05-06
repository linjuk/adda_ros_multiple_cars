from functions import \
    read_csv_fast, \
    interpolate_dist, \
    calculate_covariance_for_class, \
    probability, \
    belief_update, \
    simple_probability, \
    simple_belief_update, \
    recognize_map, \
    clear_plot
from tabulate import tabulate
from matplotlib.collections import PatchCollection
import numpy as np
import glob
import math
import random
import sys

import matplotlib.pyplot as plt
plt.style.use("seaborn")

"""
Step 1: Recognize map
"""
# map_type = recognize_map('maps/testmap.png') # X-intersection
# map_type = recognize_map('maps/testmap6_0_.png') # T-intersection
# map_type = 'x-intersection'
map_type = 't-intersection'

"""
Step 2:
- Define number of point for interpolation purposes throughout this program
- Load all trajectory files in file dictionary
- Select a test trajectory and interpolate it
"""

# set number of points
Number_Of_Points=100

# fill files directionary with file paths of all csv files
files_dictionary = {

    'left': glob.glob('trajectories/left_*.csv'),         # return all files starting with left_ in the folder
    'right': glob.glob('trajectories/right_*.csv'),       # return all files starting with right in the folder
    'straight': glob.glob('trajectories/straight_*.csv'), # return all files starting with straight in the folder
}

random_trajectory = read_csv_fast('trajectories/right_09.csv')
interpolated_random_trajectory = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], Number_Of_Points)

random_trajectory = np.asarray(random_trajectory)
[xn, yn] = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], Number_Of_Points)
random_trajectory = np.vstack((xn, yn)).T

"""
Step 3: Accumulate and plot all trajectories
"""
trajectory_csv_data = {}            # container dictionary for ploting graphs
trajectory_csv_file_wise_data = {}  # container dictionary for averaging
all_points_from_files = []          # big array container for all points from all trajectories

# color map for graph
color_map = {
    'left': 'green',
    'right': 'blue',
    'straight': 'magenta'
}

# plot graph
fig, ax = plt.subplots()
plt.figure(1)
plt.title('Movement Classification f = y(x)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')

# labels
plt.plot(0, 0, color='green', label='Left-Side Parking', alpha=0.4)
plt.plot(0, 0, color='blue', label='Right-Side Parking', alpha=0.4)
plt.plot(0, 0, color='magenta', label='Straight-Side Parking', alpha=0.4)

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
        if map_type == "t-intersection" and key == "straight":
            continue
        else:
            p = plt.plot(x, y, color=color_map[key], alpha=0.3)

"""
Step 4: Interpolate the accumulated trajectories in the previous step to NUMBER_POINTS defined in STEP 1
"""
all_points_from_files = np.asarray(all_points_from_files)
count_straight_files = len(files_dictionary['straight'])
count_left_files = len(files_dictionary['left'])
count_right_files = len(files_dictionary['right'])

all_straight = []
for i in range(count_straight_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
    points = np.vstack((xn, yn)).T
    all_straight.append(points)

all_rights = []
for i in range(count_straight_files, count_straight_files + count_right_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
    points = np.vstack((xn, yn)).T
    all_rights.append(points)

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
Step 5: Calculate and plot mean for each class i.e. all trajectories for that class
"""
means_straight = np.mean(all_straight, axis=0)
means_right = np.mean(all_rights, axis=0)
means_left = np.mean(all_lefts, axis=0)

if map_type != "t-intersection":
    plt.plot(means_straight[:, 0], means_straight[:, 1], color="black")

plt.plot(means_right[:, 0], means_right[:, 1], color="black")
plt.plot(means_left[:, 0], means_left[:, 1], color="black")

"""
Step 6: Calculate and plot standard deviation for each class i.e. all trajectories for that classes
"""
std_right = np.std(all_rights, axis=0)
std_straight = np.std(all_straight, axis=0)
std_left = np.std(all_lefts, axis=0)

circles_right = []
circles_straight = []
circles_left = []
for i in range(Number_Of_Points):
    circle_right = plt.Circle((means_right[i][0], means_right[i][1]), radius=2 * np.linalg.norm(std_right[i]))
    # radius = np.linalg.norm(std_right[i])
    if map_type != "t-intersection":
        circle_straight = plt.Circle((means_straight[i][0], means_straight[i][1]),
                                 radius=2 * np.linalg.norm(std_straight[i]))
    circle_left = plt.Circle((means_left[i][0], means_left[i][1]), radius=2 * np.linalg.norm(std_left[i]))

    # circle_right = Ellipse((means_right[i][0], means_right[i][1]), width=2*std_right[i][0], height=2*std_right[i][1])
    # circle_straight = Ellipse((means_straight[i][0], means_straight[i][1]), width=2*std_straight[i][0], height=2*std_straight[i][1])
    # circle_left = Ellipse((means_left[i][0], means_left[i][1]), width=2*std_left[i][0], height=2*std_left[i][1])

    # 1*std --> 68.27%, 2*std --> 95.45%, 3*std --> 99.73%.

    circles_right.append(circle_right)
    if map_type != "t-intersection":
        circles_straight.append(circle_straight)
    circles_left.append(circle_left)

p = PatchCollection(circles_right, alpha=0.3, color="red")
if map_type != "t-intersection":
    p1 = PatchCollection(circles_straight, alpha=0.3, color="green")
p2 = PatchCollection(circles_left, alpha=0.2, color="blue")

ax.add_collection(p)
if map_type != "t-intersection":
    ax.add_collection(p1)
ax.add_collection(p2)

"""
Step 7: Calculate covariance for trajectories in each class
"""
covariance_right = calculate_covariance_for_class(all_rights, Number_Of_Points)
covariance_straight = calculate_covariance_for_class(all_straight, Number_Of_Points)
covariance_left = calculate_covariance_for_class(all_lefts, Number_Of_Points)

"""
Step 8: Calculate likelihood of random test trajectory to belong to which class
"""
Pobs = []
for i in range(1, Number_Of_Points):
    Pobs.append([simple_probability(random_trajectory[i], means_left[i], covariance_left[i]),
                 simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                 simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i])])

Pobs = np.array(Pobs)

"""
Step 9: Calculate prior belief 
"""
if map_type == "t-intersection":
    prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
    b100 = np.array([prior_left, prior_right, 0.0])
    b10 = np.array([prior_left, prior_right, 0.0])
else:
    b100 = np.array([0.3, 0.3, 0.3])
    b10 = np.array([0.3, 0.3, 0.3])

"""
Step 10a: Do belief calculations based on prior belief (with scaling)
"""
print("Belief calculations (with scaling)\n")
b100_all = []
b10_all = []
t = 10

for i in range (0, Number_Of_Points):

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
Step 10b: Do belief calculations based on prior belief (Without Scaling)
"""
print("\n\nBelief calculations (without scaling)\n")
Pobs = []
t = 10

for i in range(1, Number_Of_Points):
    Pobs.append([ simple_probability(random_trajectory[i], means_left[i], covariance_left[i]),
                             simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                             simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i]) ])

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


for i in range (0, Number_Of_Points):

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
            print("10: ", b10, "100: ", b100)


"""
Step 11: Plot results of belief calculation on the graph for each timestep [without scaling]
"""
print("\n\nPlotting belief calculation (without scaling)\n")
INTERVAL = 0.001  # in seconds

for i in range(0, Number_Of_Points):
    print("step: ", i, "[left, right, straight]: ", b100_all_ws[i])

    belief_sum = b100_all_ws[i][0] + b100_all_ws[i][1] + b100_all_ws[i][2]
    plt.suptitle('[Left, Right, Straight] = ' + str(np.round(b100_all_ws[i], 3)) + '   [Sum] = ' + str(belief_sum) + '   [Step] = ' + str(i), fontsize=12, ha='left', x=0.05, y=0.98)
    plt.scatter(x=interpolated_random_trajectory[0][i], y=interpolated_random_trajectory[1][i], c="red", s=7, zorder=10)

    # commented for time being to avoid animation and get faster
    # plt.pause(INTERVAL)

    # Now clear the plot
    clear_plot()

plt.legend(loc='lower right')
# plt.show()

"""
Step 12a: Belief Comparison Plotting b100 vs b10 (with scaling)
"""

# plot graph
fig, ax = plt.subplots()
plt.figure(2)
plt.title('Belief Updates over Time (with scaling)')
plt.xlabel('Time Steps')
plt.ylabel('Belief over Class')

labels = {
    'left100': 'Belief for going left_100 steps',
    'right100': 'Belief for going right_100 steps',
    'straight100': 'Belief for going straight_100 steps',
    'left10': 'Belief for going left_10 steps',
    'right10': 'Belief for going right_10 steps',
    'straight10': 'Belief for going straight_10 steps'
}

b10_counter = 0
for i in range(0, Number_Of_Points):

    plt.plot(i, b100_all[i][0], marker=".", color="green", label=labels['left100'], alpha=0.4)
    plt.plot(i, b100_all[i][1], marker=".", color="blue", label=labels['right100'], alpha=0.4)
    plt.plot(i, b100_all[i][2], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)

    if i % t == 0:
        # plt.plot(i, b100_all[i][0], marker=".", color="green", label=labels['left100'], alpha=0.4)
        # plt.plot(i, b100_all[i][1], marker=".", color="blue", label=labels['right100'], alpha=0.4)
        # plt.plot(i, b100_all[i][2], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)

        plt.plot(i, b10_all[b10_counter][0], marker="D", color="green", label=labels['left10'], alpha=0.4)
        plt.plot(i, b10_all[b10_counter][1], marker="D", color="blue", label=labels['right10'], alpha=0.4)
        plt.plot(i, b10_all[b10_counter][2], marker="D", color="magenta", label=labels['straight10'], alpha=0.4)
        b10_counter+=1

    # ignore legend after first print
    for key in labels:
        labels[key] = "_nolegend_"

plt.legend(loc='center right')



# plt.show()

"""
Step 12b: Belief Comparison Plotting b100 vs b10 (without scaling)
"""

# plt.legend(loc='lower right')
# plot graph
fig, ax = plt.subplots()
plt.figure(3)
plt.title('Belief Updates over Time (without scaling)')
plt.xlabel('Time Steps')
plt.ylabel('Belief over Class')

labels_ws = {
    'left100': 'Belief for going left_100 steps',
    'right100': 'Belief for going right_100 steps',
    'straight100': 'Belief for going straight_100 steps',
    'left10': 'Belief for going left_10 steps',
    'right10': 'Belief for going right_10 steps',
    'straight10': 'Belief for going straight_10 steps'
}

b10_counter = 0
for i in range(0, Number_Of_Points):

    plt.plot(i, b100_all_ws[i][0], marker=".", color="green", label=labels_ws['left100'], alpha=0.4)
    plt.plot(i, b100_all_ws[i][1], marker=".", color="blue", label=labels_ws['right100'], alpha=0.4)
    plt.plot(i, b100_all_ws[i][2], marker=".", color="magenta", label=labels_ws['straight100'], alpha=0.4)

    if i % t == 0:
        plt.plot(i, b10_all_ws[b10_counter][0], marker="D", color="green", label=labels_ws['left10'], alpha=0.4)
        plt.plot(i, b10_all_ws[b10_counter][1], marker="D", color="blue", label=labels_ws['right10'], alpha=0.4)
        plt.plot(i, b10_all_ws[b10_counter][2], marker="D", color="magenta", label=labels_ws['straight10'], alpha=0.4)
        b10_counter+=1

    # ignore legend after first print
    for key in labels_ws:
        labels_ws[key] = "_nolegend_"

plt.legend(loc='center right')

"""
Show all figures
"""

plt.show()
