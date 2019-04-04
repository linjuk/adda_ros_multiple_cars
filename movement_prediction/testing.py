from functions import read_csv_fast, interpolate_dist
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

random_trajectory = read_csv_fast('trajectories/test_right.csv')
interpolated_trajectory = interpolate_dist(random_trajectory[:,1], random_trajectory[:, 2], 10)

# labels
plt.plot(0, 0)
plt.plot(random_trajectory[:,1], random_trajectory[:,2], '--ro')
plt.plot(interpolated_trajectory[0], interpolated_trajectory[1], '--bo')
plt.show()


