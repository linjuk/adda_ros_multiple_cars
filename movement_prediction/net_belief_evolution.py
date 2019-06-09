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
    # 'trajectories/right_101.csv',
    # 'trajectories/right_102.csv',
    # 'trajectories/right_103.csv',
    # 'trajectories/right_104.csv',
    # 'trajectories/right_105.csv',
    # 'trajectories/right_106.csv',
    # 'trajectories/right_107.csv',
    # 'trajectories/right_108.csv',
    # 'trajectories/right_109.csv',
    # 'trajectories/right_110.csv'
# ]

# input_trajectories = [
    # 'trajectories/test_straight.csv',
    # 'trajectories/straight_101.csv',
    # 'trajectories/straight_102.csv',
    # 'trajectories/straight_103.csv',
    # 'trajectories/straight_104.csv',
    # 'trajectories/straight_105.csv',
    # 'trajectories/straight_106.csv',
    # 'trajectories/straight_107.csv',
    # 'trajectories/straight_108.csv',
    # 'trajectories/straight_109.csv',
    # 'trajectories/straight_110.csv'
# ]

input_trajectories = [
    # 'trajectories/test_left.csv',
    # 'trajectories/left_101.csv',
    # 'trajectories/left_102.csv',
    'trajectories/left_103.csv',
    # 'trajectories/left_104.csv',
    # 'trajectories/left_105.csv',
    # 'trajectories/left_106.csv',
    # 'trajectories/left_107.csv',
    # 'trajectories/left_108.csv',
    # 'trajectories/left_109.csv',
    # 'trajectories/left_100.csv'
]

all_results = []
points = 100

dataset = comb_dataset(points)
files_dictionary = dataset[3]
all_means = calculate_mean(dataset)
all_covariances = calculate_covariance(dataset)

# map_type = recognize_map('maps/testmap.png')
map_type = 'x-intersection'
# map_type = 't-intersection'


# calculate scaling results for all input trajectories
for i in range(0, len(input_trajectories)):
    all_results.append(scaling_results_per_trajectory(map_type, input_trajectories[i], points, files_dictionary, all_means, all_covariances ))

# plot all results in one go
plot_scaling_results(all_results)
plt.show()