from functions import read_csv_fast, interpolate_dist, calculate_covariance_for_class, probability, belief_update, simple_probability, simple_belief_update
from tabulate import tabulate
import numpy as np
import glob
import math
import random
import sys

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

    'left': glob.glob('trajectories/left_*.csv'),         # return all files starting with left_ in the folder
    'right': glob.glob('trajectories/right_*.csv'),       # return all files starting with right in the folder
    'straight': glob.glob('trajectories/straight_*.csv'), # return all files starting with straight in the folder
}

random_trajectory = read_csv_fast('trajectories/test_right.csv')
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

"""
Step 3: Interpolate the accumulated trajectories in the previous step to NUMBER_POINTS defined in STEP 1
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
Step 4: Calculate and plot mean for each class i.e. all trajectories for that class
"""
means_straight = np.mean(all_straight, axis=0)
means_right = np.mean(all_rights, axis=0)
means_left = np.mean(all_lefts, axis=0)

"""
Step 5: Calculate covariance for trajectories in each class
"""
covariance_right = calculate_covariance_for_class(all_rights, Number_Of_Points)
covariance_straight = calculate_covariance_for_class(all_straight, Number_Of_Points)
covariance_left = calculate_covariance_for_class(all_lefts, Number_Of_Points)

"""
Step 6: Comparison of belief updates between 100 and 10 steps
"""

b100 = []
b100.append([0.33333, 0.33333, 0.33333])
b10 = []
b10.append([0.33333, 0.33333, 0.33333])
counter = 0
for point in range(0, Number_Of_Points): # Number_Of_Points = 100, numbers between 0 and 99)

    left_probability = math.pow(simple_probability(random_trajectory[point], means_left[point], covariance_left[point]), 0.1)
    right_probability = math.pow(simple_probability(random_trajectory[point], means_right[point], covariance_right[point]),0.1)
    straight_probability = math.pow(simple_probability(random_trajectory[point], means_straight[point], covariance_straight[point]), 0.1)

    updated_belief = simple_belief_update(b100[point][:], [left_probability, right_probability, straight_probability])
    b100.append(updated_belief)

    if point % 11 == 0: # coordinates are matching in points: 0-0, 1-11, 2-22, ..., 9-99

        left_probability = simple_probability(random_trajectory[point], means_left[point], covariance_left[point])
        right_probability= simple_probability(random_trajectory[point], means_right[point], covariance_right[point])
        straight_probability = simple_probability(random_trajectory[point], means_straight[point], covariance_straight[point])

        updated_belief = simple_belief_update(b10[counter][:], [left_probability, right_probability, straight_probability])
        b10.append(updated_belief)

        print("{} // [b10 point {}]: {}, [b100 point {}]: {}".format(random_trajectory[point], counter, b10[counter], point, b100[point]))
        counter+=1
        print("----------")