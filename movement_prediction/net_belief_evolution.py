from functions import \
    recognize_map, \
    comb_dataset, \
    calculate_mean, \
    calculate_covariance, \
    scaling_results_per_trajectory, \
    plot_scaling_results

import matplotlib.pyplot as plt
plt.style.use("seaborn")

# input_trajectories = [
    # 'trajectories/test_right.csv',
    # 'trajectories/right_01.csv',
    # 'trajectories/right_02.csv',
    # 'trajectories/right_03.csv',
    # 'trajectories/right_04.csv',
    # 'trajectories/right_05.csv',
    # 'trajectories/right_06.csv',
    # 'trajectories/right_07.csv',
    # 'trajectories/right_08.csv',
    # 'trajectories/right_09.csv',
    # 'trajectories/right_10.csv'
# ]

# input_trajectories = [
    # 'trajectories/test_straight.csv',
    # 'trajectories/straight_01.csv',
    # 'trajectories/straight_02.csv',
    # 'trajectories/straight_03.csv',
    # 'trajectories/straight_04.csv',
    # 'trajectories/straight_05.csv',
    # 'trajectories/straight_06.csv',
    # 'trajectories/straight_07.csv',
    # 'trajectories/straight_08.csv',
    # 'trajectories/straight_09.csv',
    # 'trajectories/straight_10.csv'
# ]

input_trajectories = [
    # 'trajectories/test_left.csv',
    'trajectories/left_01.csv',
    'trajectories/left_02.csv',
    'trajectories/left_03.csv',
    'trajectories/left_04.csv',
    'trajectories/left_05.csv',
    'trajectories/left_06.csv',
    'trajectories/left_07.csv',
    'trajectories/left_08.csv',
    'trajectories/left_09.csv',
    'trajectories/left_10.csv'
]

all_results = []
points = 100

dataset = comb_dataset(points)
files_dictionary = dataset[3]
all_means = calculate_mean(dataset)
all_covariances = calculate_covariance(dataset)

# map_type = recognize_map('maps/testmap.png')
# map_type = 'x-intersection'
map_type = 't-intersection'


# calculate scaling results for all input trajectories
for i in range(0, len(input_trajectories)):
    all_results.append(scaling_results_per_trajectory(map_type, input_trajectories[i], points, files_dictionary, all_means, all_covariances ))

# plot all results in one go
plot_scaling_results(all_results)
plt.show()