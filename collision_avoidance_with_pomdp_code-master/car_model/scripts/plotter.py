import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mt

mt.use("TkAgg")


def get_real_goals_split(goals):
    indexes = []
    current_goal = goals[0]
    indexes.append(0)
    for i in range(goals.shape[0]):
        if current_goal[0] != goals[i][0]:
            indexes.append(i)
            current_goal = goals[i]
    return indexes


# Get time steps
belief = np.genfromtxt("/home/albert/data")
velocity = np.genfromtxt("/home/albert/velocity")
real_goal = np.genfromtxt("/home/albert/real_goal")
ped_pos = np.genfromtxt("/home/albert/ped_pos")
car_pos = np.genfromtxt("/home/albert/car_pos")
types = np.genfromtxt("/home/albert/type")


poi = [np.array([12, 27]) for i in range(ped_pos.shape[0])]

# distance_to_ped = np.linalg.norm(ped_pos - car_pos, axis=1)
distance_to_poi = np.linalg.norm(car_pos - poi, axis=1)
distance_to_poi_from_ped = np.linalg.norm(ped_pos - poi, axis=1)

time_steps = np.arange(0, belief.shape[0])

indexes = get_real_goals_split(real_goal)
print indexes

# Binarize Goals
_goals = np.asarray([[5., 27.], [19., 27.]])
goal1 = np.zeros(real_goal.shape[0])
goal2 = np.zeros(real_goal.shape[0])
bin_goal1 = np.zeros(real_goal.shape[0])

for i in range(goal1.shape[0]):
    if real_goal[i][0] == _goals[0][0]:
        goal1[i] = belief[i][0]
    else:
        goal2[i] = 1
        goal1[i] = belief[i][1]
        bin_goal1[i] = 1

# ----------------------------------------------------------------------------------------
# ------------- Belief over Goals Plot -----------------
# ----------------------------------------------------------------------------------------
plt.figure(1)
plt.grid()
plt.xlabel("Time Steps $t$")
plt.ylabel("Probability Distribution $b(g)$")
plt.xlim([-2.0, time_steps.shape[0] + 4])

# Plot properties
plt.plot(time_steps, belief[:, 0], label="Goal 1 (left)")
plt.plot(time_steps, belief[:, 1], label="Goal 2 (right)")
# Plot real goal
for i in range(len(indexes)):
    plt.axvline(x=indexes[i], color='black', alpha=0.8, linestyle='--', ymin=0.0, ymax=1.0)
    plt.annotate("Real goal: {}".format(int(bin_goal1[indexes[i]] + 1)), (indexes[i] + 0.1, 0.85), size=9)

plt.legend()
plt.get_current_fig_manager().window.wm_geometry("+10+10")
plt.savefig("/home/albert/belief.eps")

# ----------------------------------------------------------------------------------------
# ------------- Velocity Plot -----------------
# ----------------------------------------------------------------------------------------
plt.figure(2)
plt.grid()
plt.ylim([-0.1, 1.1])
# plt.xticks(np.arange(0, time_steps.shape[0], 4))

plt.xlabel("Time Steps $t$")
plt.ylabel("Velocity $[m/s]$")
plt.plot(time_steps, velocity)
plt.get_current_fig_manager().window.wm_geometry("+650+10")
plt.savefig("/home/albert/velocity.eps")

# ----------------------------------------------------------------------------------------
# ------------- Paths Plot -----------------
# ----------------------------------------------------------------------------------------
plt.figure(3)
plt.grid()
plt.xlabel("x $[m]$")
plt.ylabel("y $[m]$")
plt.ylim([16., 37.])

# Plot pedestrian and car positions
plt.plot(ped_pos[:, 0], ped_pos[:, 1], 'x')
plt.plot(car_pos[:, 0], car_pos[:, 1], 'x')
# Print Time Steps
# for i in range(time_steps.shape[0]):
#     plt.annotate(time_steps[i], (ped_pos[i, 0]+0.05, ped_pos[i, 1]+0.05))

# Plot the goals and starting positions
plt.plot([5, 19], [27, 27], '^', label="Pedestrian Goals", markersize=10)
plt.annotate("Goal 1", (4.5, 28), size=9)
plt.annotate("Goal 2", (18.2, 28), size=9)
plt.plot(12, 35, '^', label="Car Goal", markersize=10)
plt.plot(ped_pos[0, 0], ped_pos[0, 1], 'o', label="Pedestrian Start Position")
plt.plot(car_pos[0, 0], car_pos[0, 1], 'o', label="Car Start Position", color="blue")


plt.get_current_fig_manager().window.wm_geometry("+1300+10")
plt.legend()
plt.savefig("/home/albert/path.eps")

# ----------------------------------------------------------------------------------------
# ------------- Distance to POI Plot -----------------
# ----------------------------------------------------------------------------------------
plt.figure(4)
plt.grid()
plt.xlabel("Time Steps $t$")
plt.ylabel("Distance to Point of Intersection $[m]$")

plt.xticks(np.arange(0, time_steps.shape[0], 2))
plt.plot(time_steps, distance_to_poi_from_ped, label="Pedestrian")
plt.plot(time_steps, distance_to_poi, label="Car")

plt.legend()
plt.savefig("/home/albert/distance_poi.eps")

# ----------------------------------------------------------------------------------------
# ------------- Type Plot -----------------
# ----------------------------------------------------------------------------------------
plt.figure(5)
plt.grid()
plt.xlabel("Time Steps $t$")
plt.ylabel("Probability of Type")
plt.xticks(np.arange(0, time_steps.shape[0], 4))
plt.plot(time_steps, types[:, 0], color='red', label="Untrustworthy", alpha=1.0)
plt.plot(time_steps, types[:, 1], color='green', label="Trustworthy", alpha=1.0)
plt.legend()
plt.savefig("/home/albert/type.eps")

plt.get_current_fig_manager().window.wm_geometry("+1300+600")

# ----------------------------------------------------------------------------------------
np.set_printoptions(precision=3, suppress=True)

print ("& {} & {}".format(np.mean(types[:, 0]), np.std(types[:, 0])))
print ("& {} & {}".format(np.mean(types[:, 1]), np.std(types[:, 1])))

print ("& {} & {}".format(np.mean(types[:, 0]), np.var(types[:, 0])))
print ("& {} & {}".format(np.mean(types[:, 1]), np.var(types[:, 1])))


# plt.show()
