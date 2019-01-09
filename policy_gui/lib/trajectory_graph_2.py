#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import std_msgs
import math

from scipy.stats import multivariate_normal
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import matplotlib.animation as animation
from numpy import genfromtxt

plt.style.use("seaborn")


def interpolate_dist(x, y, len_des):
    xd = np.diff(x)
    yd = np.diff(y)

    dist = np.sqrt(xd ** 2 + yd ** 2)
    u = np.cumsum(dist)
    u = np.hstack([[0], u])

    t = np.linspace(0, u.max(), len_des)

    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)

    return xn, yn

 # calculate probability for a given point and predicts which class is it likely to belong to

def probability(test_trajectory, means, std):
        # Gaussian Probability distribution
        # Dimentionality: n = 2 (x, y)
        # test_trajectory[i] => random predefined trajectory
        # means => means for each class
        # std => standart deviation for all means

        observation_probability = []
        for i in range(len(test_trajectory)):
            observation_probability.append(multivariate_normal.pdf(test_trajectory[i], means[i], std[i]))
            # print ("step prob: ", observation_probability)
        return observation_probability

def belief_update(belief, observation_probability, number_points):
    # Belief update for goals (for two goals together)
    belief_local_copy = belief

    print('belief local copy', belief_local_copy)
    belief_array = []
    numerator_array = []
    denominator_array = []
    class_counter = 0
    time_step_counter = 0

    # copy by reference = if original changes, every occurance changes
    # copy by value = if original chanfges, nothign happens

    for points in range(number_points):

        class_counter = 0
        numerator_array = []
        denominator_array = []
        for co in range(3):
            numerator = observation_probability[class_counter][time_step_counter] * belief_local_copy[class_counter]
            numerator_array.append(numerator)
            denominator_array.append(numerator)
            class_counter += 1

        belief_counter = 0
        for value in numerator_array:
            belief_local_copy[belief_counter] = value / sum(denominator_array)
            belief_counter += 1

        belief_array.append(list(belief_local_copy))
        print('updated belief', belief_local_copy)
        time_step_counter += 1

    return belief_array


files_dictionary = {
    'left': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_6.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_7.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_8.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_9.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/left_10.csv',
    ],
    'right': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_6.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_7.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_8.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_9.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/right_10.csv',
    ],
    'straight': [
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_1.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_2.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_3.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_4.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_5.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_6.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_7.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_8.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_9.csv',
        '/home/linjuk/adda_ros/src/code_lina/trajectories/straight_10.csv'
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

# plot graph
fig, ax = plt.subplots()
plt.title('Movement Classification f = y(x)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
# plt.ylim([0, 25])
# plt.xlim([0, 25])

all_points_from_files = []

# labels
plt.plot(0, 0, color='green', label='Left-Side Parking', alpha=0.4)
plt.plot(0, 0, color='blue', label='Right-Side Parking', alpha=0.4)
plt.plot(0, 0, color='magenta', label='Straight-Side Parking', alpha=0.4)

# loop over trajectories i.e. left, right, straight
for key in files_dictionary:
    trajectory_csv_file_wise_data[key] = {}
    # loop over files in each trajectory
    for index, file in enumerate(files_dictionary[key]):
        all_points_from_files.append(genfromtxt(file, dtype=float, delimiter=",", skip_header=1))
        # read file
        trajectory_csv_data[key] = (genfromtxt(file, dtype=float, delimiter=",", skip_header=1))
        # aggregate data in container for averaging
        trajectory_csv_file_wise_data[key][index] = []
        trajectory_csv_file_wise_data[key][index].append(trajectory_csv_data[key])
        # segregate x and y for plotting
        x, y = trajectory_csv_data[key][:, 1], trajectory_csv_data[key][:, 2]
        p = plt.plot(x, y, color=color_map[key], alpha=0.3)




NUMBER_POINTS = 500
all_points_from_files = np.asarray(all_points_from_files)

all_straight = []
for i in range(10):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_straight.append(points)

all_rights = []
for i in range(10, 20):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_rights.append(points)

all_lefts = []
for i in range(20, 30):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_lefts.append(points)

all_rights = np.asarray(all_rights)
all_straight = np.asarray(all_straight)
all_lefts = np.asarray(all_lefts)



means_straight = np.mean(all_straight, axis=0)
means_right = np.mean(all_rights, axis=0)
means_left = np.mean(all_lefts, axis=0)
plt.plot(means_straight[:, 0], means_straight[:, 1], color="black")
plt.plot(means_right[:, 0], means_right[:, 1], color="black")
plt.plot(means_left[:, 0], means_left[:, 1], color="black")

std_right = np.std(all_rights, axis=0)
std_straight = np.std(all_straight, axis=0)
std_left = np.std(all_lefts, axis=0)

circles_right = []
circles_straight = []
circles_left = []
for i in range(NUMBER_POINTS):
    circle_right = plt.Circle((means_right[i][0], means_right[i][1]), radius=2*np.linalg.norm(std_right[i]))
    circle_straight = plt.Circle((means_straight[i][0], means_straight[i][1]), radius=2*np.linalg.norm(std_straight[i]))
    circle_left = plt.Circle((means_left[i][0], means_left[i][1]), radius=2*np.linalg.norm(std_left[i]))

    circles_right.append(circle_right)
    circles_straight.append(circle_straight)
    circles_left.append(circle_left)

p = PatchCollection(circles_right, alpha=0.03, color="red")
p1 = PatchCollection(circles_straight, alpha=0.03, color="green")
p2 = PatchCollection(circles_left, alpha=0.02, color="blue")
ax.add_collection(p)
ax.add_collection(p1)
ax.add_collection(p2)


# for i in range(50):
#     sample_x = np.random.multivariate_normal(means_right[:, 0], np.dot(std_right, std_right.T))
#     sample_y = np.random.multivariate_normal(means_right[:, 1], np.dot(std_right, std_right.T))
#     # plt.scatter(sample_x, sample_y, marker=".", c="green", alpha=0.1)
#     plt.plot(sample_x, sample_y, marker=".", c="green", alpha=0.1)




 # select a random trajectory and interpolate random trajectory to 500 points
random_trajectory = genfromtxt(files_dictionary['right'][0], dtype=float, delimiter=",", skip_header=1)


# interpolate random trajectory
interpolated_random_trajectory = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], NUMBER_POINTS)
print("interpolated_random_trajectory")
print(interpolated_random_trajectory)

# plot to graph
plt.plot(interpolated_random_trajectory[0], interpolated_random_trajectory[1], color="white")

# def animate(i, data):
#     p = sns.lineplot(x=data, y=data, data=data, color="r")
#     p.tick_params(labelsize=17)
#     plt.setp(p.lines,linewidth=7)
#
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=20, metadata=dict(artist='Me'), bitrate=1800)


random_trajectory = np.asarray(random_trajectory)
[xn, yn] = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], 500)
random_trajectory = np.vstack((xn, yn)).T

# pdf calculation
straight_probability = probability(random_trajectory, means_straight, std_straight)
print('straight probability: ', straight_probability)
left_probability = probability(random_trajectory, means_left, std_left)
print('left probability: ', left_probability)
right_probability = probability(random_trajectory, means_right, std_right)
print('right probability: ', right_probability)


# belief calculation
belief = []

belief.append(0.33333) # prob that class1
belief.append(0.33333) # prob that class2
belief.append(0.33333) # prob that class3

# copy by reference
# a = [1,2,3,4]
# b = []
# b.append(a)
# b.append(a)
# value of b is [[1,2,3,4],[1,2,3,4]]
# a[0] = 100
# now value of b is [[100,2,3,4],[100,2,3,4]]

# to avoid this problem of copy by reference, we use copy by value with creates separate copies instead of links
# b.append(list(a))

trajectory_beliefs = belief_update(belief, [straight_probability, left_probability, right_probability], NUMBER_POINTS)
print("step belief[straight, left, right]", trajectory_beliefs)
# belief = step_belief

plt.legend(loc='lower right')
plt.show()