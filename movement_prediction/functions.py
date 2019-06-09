import cv2
from scipy.stats import multivariate_normal
from scipy.interpolate import NearestNDInterpolator
# from dtw import dtw
import numpy as np
import pandas as pd
import glob
import matplotlib.pyplot as plt
import math
import random
import sys
from matplotlib.collections import PatchCollection
import os

# Sets plot style to given theme
plt.style.use("seaborn")

### ORDER OF DIRECTIONS
### RIGHT, STRAIGHT, LEFT => applies to new functions only

def replace_zeros_in_list(list):
    """
    Takes in a list, replaces zero with really small values to avoid divide by zero or cascade effect
    :param list:
    :return: list
    """
    for (i, item) in enumerate(list):
        if item == 0:
            list[i] = 2.2250738585072014e-308
    return list

def read_csv_fast(file):
    """
    Faster way to read csv files using Pandas since numpy's csv reading functions (genfromtxt and loadtxt) are very slow especially when number of
    files is larger
    """
    return pd.read_csv(file, skiprows=1).values

def recognize_map(map_file):
    """
       Given a type of map file, try to recognize using geometric operations which
       type of map it is.
       """

    img1 = cv2.imread(map_file)
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    ret1, thresh1 = cv2.threshold(gray1, 127, 255, 1)
    h1, contours1, h1 = cv2.findContours(thresh1, 1, 2)
    x = 0
    for cnt1 in contours1:
        approx1 = cv2.approxPolyDP(cnt1, 0.01 * cv2.arcLength(cnt1, True), True)

        if len(approx1) == 4:
            x = x + 1
            cv2.drawContours(img1, [cnt1], 0, (0, 0, 255), -1)

    if x == 8:
        print("This Image is X-intersection")
        return 'x-intersection'

    elif x == 4:
        print("This Image is T-intersection")
        return 't-intersection'
    else:
        return 'unknown'

def clear_plot():
    """
    Clears the graph plot after each interval
    """
    plt.title('Movement Classification f = y(x)')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

def interpolate_dist(x, y, len_des):
    """
    Takes in a trajectory and interpolates it to specified number of points.
    For example 1000 point to 100 or 10.
    """

    xd = np.diff(x)
    yd = np.diff(y)

    dist = np.sqrt(xd ** 2 + yd ** 2)
    u = np.cumsum(dist)
    u = np.hstack([[0], u])

    t = np.linspace(0, u.max(), len_des)

    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)

    return xn, yn

# calculated using the last formula on the formulas paper
def calculate_covariance_for_class(trajectories_in_class, number_points):
    """
    Given all trajectories in a class, this function calculates
    covariance against all trajectories in each class
    """
    cov_for_all_timesteps = []
    for point in range(number_points):
        calculate_cov_on = []
        for trajectory in range(len(trajectories_in_class)):
            calculate_cov_on.append(trajectories_in_class[trajectory][point])

        cov_for_this_timestep = np.cov(calculate_cov_on, rowvar=False)
        cov_for_all_timesteps.append(cov_for_this_timestep)
    return cov_for_all_timesteps

# simple probability formula for one point instead of full trajectory like
def simple_probability(point, mean, covariance):
    return multivariate_normal.pdf(point, mean, covariance)

# calculated using the pdf formula, 2) equation on the formulas paper
def probability(test_trajectory, mean_of_all_classes, covariance):
    """
    Given a test trajectory, this function calculates probability density across
    different classes (right, left or straight) for each point based on mean and covariance
    of the trajectory
    """
    # array to collect probabilities for each point
    observation_probability = []

    # loop over each point and calculate PDF
    for point_in_test_trajectory in range(len(test_trajectory)):
        # calculate pdf for single time step
        pdf_for_this_timestamp = multivariate_normal.pdf(test_trajectory[point_in_test_trajectory],
                                                         mean_of_all_classes[point_in_test_trajectory],
                                                         covariance[point_in_test_trajectory])
        # append calculated pdf to the master array defined above
        observation_probability.append(pdf_for_this_timestamp)
    return observation_probability

def probability_with_power(test_trajectory, mean_of_all_classes, covariance, power):
    """
    Clone of above function for testing purposes, simply applies power on each PDF.

    Given a test trajectory, this function calculates probability density across
    different classes (right, left or straight) for each point based on mean and covariance
    of the trajectory
    """
    # array to collect probabilities for each point
    observation_probability = []

    # loop over each point and calculate PDF
    for point_in_test_trajectory in range(len(test_trajectory)):
        # calculate pdf for single time step
        pdf_for_this_timestamp = multivariate_normal.pdf(test_trajectory[point_in_test_trajectory],
                                                         mean_of_all_classes[point_in_test_trajectory],
                                                         covariance[point_in_test_trajectory])
        # skip power for first step
        if point_in_test_trajectory == 0:
            observation_probability.append(pdf_for_this_timestamp)
        elif point_in_test_trajectory > 0:
            # append calculated pdf to the master array defined above
            observation_probability.append(math.pow(pdf_for_this_timestamp, power))
    return observation_probability

def sigma():
    """
    This function signifies noise in PDF function. Previously used for
    calculating covariance
    """
    identitymatrix = np.eye(2)
    factor = 1
    sigma = identitymatrix * factor
    return sigma

# calculated using the 1) quation on the formulas paper
def simple_belief_update(belief_input, observation_probability):

    # Belief update for goals (for two goals together)
    belief_local_copy = belief_input
    numerator_array = []
    denominator_array = []

    for class_counter in range(len(observation_probability)):
        numerator = observation_probability[class_counter] * belief_local_copy[class_counter]
        numerator_array.append(numerator)
        denominator_array.append(numerator)

    for belief_counter in range(len(numerator_array)):
        # belief_local_copy[belief_counter] = float("{0:.2f}".format(value / sum(denominator_array)))
        belief_local_copy[belief_counter] = 0.00001 if (numerator_array[belief_counter] / sum(denominator_array)) == 0 else numerator_array[belief_counter] / sum(denominator_array)

    return belief_local_copy

# calculated using the 1) quation on the formulas paper
def belief_update(belief_input, observation_probability, number_points):
    """
    This function returns the newly calculated belief given inputs of
    - Previous belief
    - Observation probability for each class i.e left, right and straight
    - Number of points
    """
    # Belief update for goals (for two goals together)
    belief_local_copy = belief_input

    # print('belief local copy', belief_local_copy)

    belief_array = []
    # comment if you only want seeding beleif to be 0.3, 0.3, 0.3 not the updated for step 0
    belief_array.append(list(belief_local_copy))

    for points in range(0, number_points):

        numerator_array = []
        denominator_array = []

        for class_counter in range(3):
            numerator = observation_probability[class_counter][points] * belief_local_copy[class_counter]
            numerator_array.append(numerator)
            denominator_array.append(numerator)

        for belief_counter in range(len(numerator_array)):
            # belief_local_copy[belief_counter] = float("{0:.2f}".format(value / sum(denominator_array)))
            belief_local_copy[belief_counter] = 0.00001 if (numerator_array[belief_counter] / sum(denominator_array)) == 0 else numerator_array[belief_counter] / sum(denominator_array)

        belief_array.append(list(belief_local_copy))

    return belief_array

def distance_formula(p1, p2):
    return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))

def distance_formula_dtw(p1, p2):
    return dtw.distance(p1, p2)

def comb_dataset(Number_Of_Points):
    """
    Go through all trajectory files of dataset, interpolate each and return accumulated values
    """
    # Load and accumulate all files
    trajectory_csv_data = {}  # container dictionary for ploting graphs
    trajectory_csv_file_wise_data = {}  # container dictionary for averaging
    all_points_from_files = []  # big array container for all points from all trajectories

    # fill files directionary with file paths of all csv files
    files_dictionary = {
        'right': glob.glob(os.path.dirname(os.path.realpath(__file__))+'/trajectories/right_*.csv'),  # return all files starting with right in the folder
        'straight': glob.glob(os.path.dirname(os.path.realpath(__file__))+'/trajectories/straight_*.csv'),  # return all files starting with straight in the folder
        'left': glob.glob(os.path.dirname(os.path.realpath(__file__)) + '/trajectories/left_*.csv') # return all files starting with left_ in the folder

    }

    # loop over trajectories i.e. left, right, straight
    for key in files_dictionary:

        trajectory_csv_file_wise_data[key] = {}

        for index, file in enumerate(files_dictionary[key]):  # loop over files in each trajectory
            file_raw_data = read_csv_fast(file)
            all_points_from_files.append(file_raw_data)
            trajectory_csv_data[key] = (file_raw_data)  # read file
            trajectory_csv_file_wise_data[key][index] = []  # aggregate data in container for averaging
            trajectory_csv_file_wise_data[key][index].append(trajectory_csv_data[key])

    # Interpolate the accumulated trajectories
    all_points_from_files = np.asarray(all_points_from_files)
    count_right_files = len(files_dictionary['right'])
    count_straight_files = len(files_dictionary['straight'])
    count_left_files = len(files_dictionary['left'])

    print('Count of dataset')
    print('right', count_right_files)
    print('left', count_left_files)
    print('straight', count_straight_files)

    all_rights = []
    for i in range(count_straight_files, count_straight_files + count_right_files):
        [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
        points = np.vstack((xn, yn)).T
        all_rights.append(points)

    all_straights = []
    for i in range(count_straight_files):
        [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
        points = np.vstack((xn, yn)).T
        all_straights.append(points)

    all_lefts = []
    for i in range(count_straight_files + count_right_files,
                   count_straight_files + count_right_files + count_left_files):
        [xn, yn] = interpolate_dist(all_points_from_files[i][:, 1], all_points_from_files[i][:, 2], Number_Of_Points)
        points = np.vstack((xn, yn)).T
        all_lefts.append(points)

    all_rights = np.asarray(all_rights)
    all_straights = np.asarray(all_straights)
    all_lefts = np.asarray(all_lefts)


    #Return interpolated values for all trajectory directions, files dictonary and number of points for later use
    return all_rights, all_straights, all_lefts, files_dictionary, Number_Of_Points

def calculate_mean(dataset):
    """
    Calculate mean using accumulated and interpolated values of all directions from dataset
    """
    # destructure input dataset
    all_rights, all_straights, all_lefts, files_dictionary, Number_Of_Points = dataset

    # calculate mean
    means_right = np.mean(all_rights, axis=0)
    means_straights = np.mean(all_straights, axis=0)
    means_left = np.mean(all_lefts, axis=0)

    # return calculated mean
    return means_right, means_straights, means_left

def calculate_covariance(dataset):
    """
    Based on interpolaed and accumulated dataset, calculate covariance for each direction
    """
    all_rights,  all_straights, all_lefts, files_dictionary, Number_Of_Points = dataset
    covariance_right = calculate_covariance_for_class(all_rights, Number_Of_Points)
    covariance_straight = calculate_covariance_for_class(all_straights, Number_Of_Points)
    covariance_left = calculate_covariance_for_class(all_lefts, Number_Of_Points)
    return covariance_right, covariance_straight, covariance_left

def calculate_std(dataset):
    """
    Based on interpolaed and accumulated dataset, calculate covariance for each direction
    """
    all_rights, all_straights, all_lefts, files_dictionary, Number_Of_Points = dataset
    std_right = np.std(all_rights, axis=0)
    std_straight = np.std(all_straights, axis=0)
    std_left = np.std(all_lefts, axis=0)
    return std_right, std_straight, std_left

def scaling_results_per_trajectory(map_type, input_trajectory, Number_Of_Points, files_dictionary, all_means, all_variance):
    """

    :param map_type:
    :param input_trajectory:
    :param Number_Of_Points:
    :param files_dictionary:
    :param all_means:
    :param all_variance:
    :return:
    """

    """
    Step 1: Read input trajectory and interpolate it for further use
    """

    random_trajectory = read_csv_fast(input_trajectory)
    random_trajectory = np.asarray(random_trajectory)
    [xn, yn] = interpolate_dist(random_trajectory[:, 1], random_trajectory[:, 2], Number_Of_Points)
    random_trajectory = np.vstack((xn, yn)).T

    """
    Step 2: Destructure input for means and variance
    """
    means_right, means_straight, means_left = all_means
    covariance_right, covariance_straight, covariance_left  = all_variance

    """
    Step 3: Calculate likelihood of random test trajectory to belong to which class
    """
    Pobs = []
    for i in range(1, Number_Of_Points):
        Pobs.append([simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                     simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i]),
                     simple_probability(random_trajectory[i], means_left[i], covariance_left[i])])

    Pobs = np.array(Pobs)

    """
    Step 4: Calculate prior belief 
    """
    if map_type == "t-intersection":
        prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        b100 = np.array([prior_right, 0.0, prior_left])
        b10 = np.array([prior_right, 0.0, prior_left])
    else:
        b100 = np.array([0.3, 0.3, 0.3])
        b10 = np.array([0.3, 0.3, 0.3])

    """
    Step 5: Do belief calculations based on prior belief (with scaling)
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
    Step 6: Do belief calculations based on prior belief (Without Scaling)
    """
    print("\n\nBelief calculations (without scaling)\n")
    Pobs = []
    t = 10

    for i in range(1, Number_Of_Points):
        Pobs.append([ simple_probability(random_trajectory[i], means_right[i], covariance_right[i]),
                      simple_probability(random_trajectory[i], means_straight[i], covariance_straight[i]),
                      simple_probability(random_trajectory[i], means_left[i], covariance_left[i]) ])

    Pobs = np.array(Pobs)

    # initialize prior based on recognized map type
    if map_type == "t-intersection":
        prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        b100 = np.array([prior_right, 0.0, prior_left])
        b10 = np.array([prior_right, 0.0, prior_left])
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

    # return stuff
    return b100_all, b10_all, b100_all_ws, b10_all_ws, Number_Of_Points, t

def plot_scaling_results(all_results):
    """

    :param all_results:
    :return:
    """

    # plot graph
    fig, ax = plt.subplots()
    plt.figure(1)
    plt.title('Belief Updates over Time')
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



    for rs in range(0, len(all_results)):
        # destructure input for each trajectory
        b100_all, b10_all, b100_all_ws, b10_all_ws, Number_Of_Points, t = all_results[rs]
        b10_counter = 0

        # print b100_all[:, 1]
        # x_array = range(Number_Of_Points)
        # plt.plot(x_array, b100_all[:,0], color="blue", label=labels['right100'], alpha=0.4)
        # plt.plot(x_array, b100_all[:,1], color="magenta", label=labels['straight100'], alpha=0.4)
        # plt.plot(x_array, b100_all[:,2], color="magenta", label=labels['left100'], alpha=0.4)


        for i in range(0, Number_Of_Points):

            plt.plot(i, b100_all[i][0], marker=".", color="blue", label=labels['right100'], alpha=0.4)
            plt.plot(i, b100_all[i][1], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)
            plt.plot(i, b100_all[i][2], marker=".", color="green", label=labels['left100'], alpha=0.4)
            #
            # plt.plot(i, b100_all[i][0], color="blue", label=labels['right100'], alpha=0.4)
            # plt.plot(i, b100_all[i][1], color="magenta", label=labels['straight100'], alpha=0.4)
            # plt.plot(i, b100_all[i][2], color="green", label=labels['left100'], alpha=0.4)

            if i % t == 0:
                # plt.plot(i, b100_all[i][0], marker=".", color="blue", label=labels['right100'], alpha=0.4)
                # plt.plot(i, b100_all[i][1], marker=".", color="magenta", label=labels['straight100'], alpha=0.4)
                # plt.plot(i, b100_all[i][2], marker=".", color="green", label=labels['left100'], alpha=0.4)

                # plt.plot(i, b10_all[b10_counter][0], marker="D", color="blue", label=labels['right10'], alpha=0.4)
                # plt.plot(i, b10_all[b10_counter][1], marker="D", color="magenta", label=labels['straight10'], alpha=0.4)
                # plt.plot(i, b10_all[b10_counter][2], marker="D", color="green", label=labels['left10'], alpha=0.4)
                b10_counter += 1

            # ignore legend after first print
            for key in labels:
                labels[key] = "_nolegend_"


        plt.legend(loc='center right')



    # plt.show()

    # """
    # Step 2: Belief Comparison Plotting b100 vs b10 (without scaling)
    # """
    #
    # # plt.legend(loc='lower right')
    # # plot graph
    # fig, ax = plt.subplots()
    # plt.figure(2)
    # plt.title('Belief Updates over Time (without scaling)')
    # plt.xlabel('Time Steps')
    # plt.ylabel('Belief over Class')
    #
    # labels_ws = {
    #     'left100': 'Belief for going left_100 steps',
    #     'right100': 'Belief for going right_100 steps',
    #     'straight100': 'Belief for going straight_100 steps',
    #     'left10': 'Belief for going left_10 steps',
    #     'right10': 'Belief for going right_10 steps',
    #     'straight10': 'Belief for going straight_10 steps'
    # }
    #
    # for rs in range(0, len(all_results)):
    #     b100_all, b10_all, b100_all_ws, b10_all_ws, Number_Of_Points, t = all_results[rs]
    #
    #     b10_counter = 0
    #     for i in range(0, Number_Of_Points):
    #
    #         plt.plot(i, b100_all_ws[i][0], marker=".", color="blue", label=labels_ws['right100'], alpha=0.4)
    #         plt.plot(i, b100_all_ws[i][1], marker=".", color="magenta", label=labels_ws['straight100'], alpha=0.4)
    #         plt.plot(i, b100_all_ws[i][2], marker=".", color="green", label=labels_ws['left100'], alpha=0.4)
    #
    #         if i % t == 0:
    #             plt.plot(i, b10_all_ws[b10_counter][0], marker="D", color="blue", label=labels_ws['right10'], alpha=0.4)
    #             plt.plot(i, b10_all_ws[b10_counter][1], marker="D", color="magenta", label=labels_ws['straight10'], alpha=0.4)
    #             plt.plot(i, b10_all_ws[b10_counter][2], marker="D", color="green", label=labels_ws['left10'],
    #                      alpha=0.4)
    #             b10_counter += 1
    #
    #         # ignore legend after first print
    #         for key in labels_ws:
    #             labels_ws[key] = "_nolegend_"
    #
    #     plt.legend(loc='center right')

def belief_update_for_rviz(input_cordinates, mean, covariance, belief_array, map_type, files_dictionary):

    mean =  np.array(mean)
    covariance = np.array(covariance)

    # prior based on map type
    if(map_type == 't-intersection'):
        print('Calculating belief for t-intersection map')
        #  probability
        Pobs = []
        Pobs.append([simple_probability(input_cordinates, mean[0], covariance[0]),
                     0,
                     simple_probability(input_cordinates, mean[2], covariance[2])])
        Pobs = np.array(Pobs)


        prior_right = len(files_dictionary['right']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        prior_left = len(files_dictionary['left']) / float(len(files_dictionary['left']) + len(files_dictionary['right']))
        prior_belief = np.array([prior_right, 0.0, prior_left])


    elif(map_type == 'x-intersection'):
        #  probability
        Pobs = []
        Pobs.append([simple_probability(input_cordinates, mean[0], covariance[0]),
                     simple_probability(input_cordinates, mean[1], covariance[1]),
                     simple_probability(input_cordinates, mean[2], covariance[2])])
        Pobs = np.array(Pobs)

        prior_belief = np.array([0.3, 0.3, 0.3])

    # beleif without scaling
    if len(belief_array) == 0:
        return prior_belief

    else:
        P = Pobs[0, :]
        # print("From function.py: P", P)
        # print('from function.py: belief arrray', belief_array)
        belief = belief_array[len(belief_array)-1] * P
        return belief / np.sum(replace_zeros_in_list(belief))


