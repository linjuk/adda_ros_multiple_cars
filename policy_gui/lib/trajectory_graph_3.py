#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import std_msgs
import math
import random
import cv2
import glob
import keyboard

from scipy.stats import multivariate_normal
from scipy.spatial import distance
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import matplotlib.animation as animation
from numpy import genfromtxt

plt.style.use("seaborn")

def clear_plot(title, fig_num):

    # Create new plot
    # plt.figure(fig_num)
    plt.title('Movement Classification f = y(x)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # plt.ylim([0, 25])
    # plt.xlim([0, 25])


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

def probability(test_trajectory, means, sigma):

        observation_probability = []
        for i in range(len(test_trajectory)):
            # observation_probability.append(multivariate_normal.pdf(test_trajectory[i], means[i], std[i]))
            observation_probability.append(multivariate_normal.pdf(test_trajectory[i], means[i], sigma))
            print ("step prob: ", observation_probability)
        return observation_probability

def sigma():

    identitymatrix = np.eye(2)
    factor = 1
    sigma = identitymatrix * factor
    return sigma

def belief_update(belief, observation_probability, number_points):
    # Belief update for goals (for two goals together)
    belief_local_copy = belief

    print('belief local copy', belief_local_copy)
    belief_array = []
    numerator_array = []
    denominator_array = []
    class_counter = 0
    time_step_counter = 0

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
            # belief_local_copy[belief_counter] = float("{0:.2f}".format(value / sum(denominator_array)))
            belief_local_copy[belief_counter] = 0.00001 if (value / sum(denominator_array)) == 0 else value / sum(denominator_array)
            belief_counter += 1

        belief_array.append(list(belief_local_copy))
        # print('updated belief', belief_local_copy)
        time_step_counter += 1

    return belief_array
    # return np.where(belief_array == 0, 0.1, belief_array)

files_dictionary = {
    'left': glob.glob('/home/linjuk/adda_ros/src/code_lina/trajectories/left_*.csv'),
    'right': glob.glob('/home/linjuk/adda_ros/src/code_lina/trajectories/right_*.csv'),
    'straight': glob.glob('/home/linjuk/adda_ros/src/code_lina/trajectories/straight_*.csv'),
}


#################################################################
## 1st step: checking similarities                             ##
## on basis of No. of specific geometries found in the picture ##
#################################################################

img1 = cv2.imread('testmap5.png')
img2 = cv2.imread('testmap6_0_.png')
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
ret1, thresh1 = cv2.threshold(gray1, 127, 255, 1)
ret2, thresh2 = cv2.threshold(gray2, 127, 255, 1)
img1, contours1, h1 = cv2.findContours(thresh1, 1, 2)
img2, contours2, h2 = cv2.findContours(thresh2, 1, 2)
x = 0
y = 0

for cnt1 in contours1:
    approx1 = cv2.approxPolyDP(cnt1, 0.01 * cv2.arcLength(cnt1, True), True)

    if len(approx1) == 4:
        x = x + 1
        cv2.drawContours(img1, [cnt1], 0, (0, 0, 255), -1)

for cnt2 in contours2:
    approx2 = cv2.approxPolyDP(cnt2, 0.01 * cv2.arcLength(cnt2, True), True)

    if len(approx2) == 4:
        y = y + 1
        cv2.drawContours(img2, [cnt2], 0, (0, 0, 255), -1)

print("x = ", x)
print("y = ", y)

if x > y:
    cv2.waitKey(0)
    cv2.imwrite('X.png', img1)
    cv2.imwrite('T.png', img2)
    cv2.destroyAllWindows()
elif x < y:
    cv2.waitKey(0)
    cv2.imwrite('T.png', img1)
    cv2.imwrite('X.png', img2)
    cv2.destroyAllWindows()


######################################################################################
## 1_1st step: map recognition                                                      ##
## on basis of No. of specific geometries found in the picture in the previous step ##
######################################################################################

img1 = cv2.imread('/home/linjuk/adda_ros/src/code_lina/simple_sim_car/pomdp_car_launch/maps/testmap.png')
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
ret1, thresh1 = cv2.threshold(gray1, 127, 255, 1)
im1, contours1, h1 = cv2.findContours(thresh1, 1, 2)
x = 0

for cnt1 in contours1:
    approx1 = cv2.approxPolyDP(cnt1, 0.01 * cv2.arcLength(cnt1, True), True)

    if len(approx1) == 4:
        x = x + 1
        cv2.drawContours(img1, [cnt1], 0, (0, 0, 255), -1)

if x == 8:
    print("This Image is X-intersection")

elif x == 4:
    print("This Image is T-intersection")



#################################
## 2nd step: prediction making ##
#################################



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
plt.figure(1)
plt.title('Movement Classification f = y(x)')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
# plt.ylim([0, 25])
# plt.xlim([0, 25])



all_points_from_files = []

# labels
# plt.plot(0, 0, color='green', label='Left-Side Parking', alpha=0.4)
# plt.plot(0, 0, color='blue', label='Right-Side Parking', alpha=0.4)
# plt.plot(0, 0, color='magenta', label='Straight-Side Parking', alpha=0.4)

# loop over trajectories i.e. left, right, straight
for key in files_dictionary:
    print(key)
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
        # p = plt.plot(x, y, color=color_map[key], alpha=0.3)




NUMBER_POINTS = 20
all_points_from_files = np.asarray(all_points_from_files)

count_straight_files = len(files_dictionary['straight'])
count_left_files = len(files_dictionary['left'])
count_right_files = len(files_dictionary['right'])

all_straight = []
for i in range(count_straight_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_straight.append(points)

all_rights = []
for i in range(count_straight_files, count_straight_files + count_right_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_rights.append(points)

all_lefts = []
for i in range(count_straight_files + count_right_files, count_straight_files + count_right_files + count_left_files):
    [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], NUMBER_POINTS)
    points = np.vstack((xn, yn)).T
    all_lefts.append(points)

all_rights = np.asarray(all_rights)
all_straight = np.asarray(all_straight)
all_lefts = np.asarray(all_lefts)

def mue(trajectory_point, goal, velocity = 1, time_delta = 1):
    mue_result = trajectory_point + ((np.array(goal) - np.array(trajectory_point))/(distance.euclidean(goal, trajectory_point))) * velocity * time_delta
    return mue_result

def mue_array_process(trajectory_array, goal):
    mue_array = []
    for index in range(NUMBER_POINTS):
        trajectory_point = [trajectory_array[0][index], trajectory_array[1][index]]
        mue_result = mue(trajectory_point, goal)
        mue_array.append(mue_result)
    return mue_array

means_straight = np.mean(all_straight, axis=0)
means_right = np.mean(all_rights, axis=0)
means_left = np.mean(all_lefts, axis=0)


# print(interpolated_random_trajectory)
# mues_straight = mue_array_process(interpolated_random_trajectory, [14.5, 23])
# mues_right = mue_array_process(interpolated_random_trajectory, [24, 11.5])
# mues_left = mue_array_process(interpolated_random_trajectory, [1, 14])

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

p = PatchCollection(circles_right, alpha=0.3, color="red")
p1 = PatchCollection(circles_straight, alpha=0.3, color="green")
p2 = PatchCollection(circles_left, alpha=0.2, color="blue")
ax.add_collection(p)
ax.add_collection(p1)
ax.add_collection(p2)

# select a random trajectory and interpolate random trajectory to 500 points

# sample_trajectory_categories = ['straight', 'left', 'right']
# random_trajectory = genfromtxt(files_dictionary[random.choice(sample_trajectory_categories)][np.random.randint(0, 9)], dtype=float, delimiter=",", skip_header=1)
random_trajectory = genfromtxt( '/home/linjuk/adda_ros/src/code_lina/trajectories/test.csv', dtype=float, delimiter=",", skip_header=1)
# interpolate random trajectory
interpolated_random_trajectory = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], NUMBER_POINTS)
# print("interpolated_random_trajectory")
# print(interpolated_random_trajectory)

# plot to graph
# commented in favour of animating scatter
# plt.plot(interpolated_random_trajectory[0], interpolated_random_trajectory[1], color="white")

random_trajectory = np.asarray(random_trajectory)
[xn, yn] = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], NUMBER_POINTS)
random_trajectory = np.vstack((xn, yn)).T

mues_straight = mue_array_process(interpolated_random_trajectory, [14.5, 23])
mues_right = mue_array_process(interpolated_random_trajectory, [24, 11.5])
mues_left = mue_array_process(interpolated_random_trajectory, [1, 14])

# pdf calculation
straight_probability = probability(random_trajectory, mues_straight, sigma())
# straight_probability = probability(random_trajectory, means_straight, std_straight)
print('straight probability: ', straight_probability)
left_probability = probability(random_trajectory, mues_left, sigma())
# left_probability = probability(random_trajectory, means_left, std_left)
print('left probability: ', left_probability)
right_probability = probability(random_trajectory, mues_right, sigma())
# right_probability = probability(random_trajectory, means_right, std_right)
print('right probability: ', right_probability)


# belief calculation
belief = []

belief.append(0.33333) # prob that class1
belief.append(0.33333) # prob that class2
belief.append(0.33333) # prob that class3

trajectory_beliefs = belief_update(belief, [left_probability, right_probability, straight_probability], NUMBER_POINTS)

# animate my graph
INTERVAL = 0.001  # in seconds
for point in range(0, NUMBER_POINTS):

    # if keyboard.is_pressed('c'):
        belief_sum = trajectory_beliefs[point][0] + trajectory_beliefs[point][1] + trajectory_beliefs[point][2]
        print("[Left, Right, Straight] = " + str(trajectory_beliefs[point]), "Sum: ", belief_sum, "; Step: ", point)
        # plt.suptitle('[Left, Right, Straight] = ' + str(trajectory_beliefs[point]) + '   [Sum] = ' + str(belief_sum) + '   [Step] = ' + str(point), fontsize=12,  ha='left', x=0.05, y=0.98)
        plt.suptitle('[Left, Right, Straight] = ' + str(np.round(trajectory_beliefs[point], 3)) + '   [Sum] = ' + str(belief_sum) + '   [Step] = ' + str(point), fontsize=12, ha='left', x=0.05, y=0.98)
        plt.scatter(x=interpolated_random_trajectory[0][point], y=interpolated_random_trajectory[1][point], c="white", s=7, zorder=10)
        # This will handle the "animation" automatically by "pausing"
        plt.pause(INTERVAL)

        # Now clear the plot
        clear_plot("Movement Classification f = y(x)", 1)
    # else:
    #     pass

print("step belief[straight, left, right]", trajectory_beliefs)
# belief = step_belief

plt.legend(loc='lower right')
plt.show()