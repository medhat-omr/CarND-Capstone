import pandas as pd
import matplotlib.pyplot as plt
import re

bag_folder = 'csv_files/'

# /base_waypoints topic
# published once, hence use base waypoints at the first timestep
wp = pd.read_csv(bag_folder + 'base_waypoints.csv')
base_waypoints = dict()
for col in wp:
    if "position" in col:
        waypoint_idx = col.split(".")[1]
        result = re.search("\d+", waypoint_idx)
        idx_key = int(result.group())
        if idx_key not in base_waypoints:
            base_waypoints[idx_key] = dict()
        base_waypoints[idx_key][col[-1]] = wp[col][0]

# /final_waypoints topic
# we need only to get first final waypoint at each time step
wp = pd.read_csv(bag_folder + 'final_waypoints.csv')
final_waypoints = dict()
for col in wp:
    if "position" in col:
        waypoint_idx = col.split(".")[1]
        result = re.search("\d+", waypoint_idx)
        idx_key = int(result.group())
        if idx_key != 0:
            continue
        final_waypoints[col[-1]] = wp[col]

# /current_pose topic
wp = pd.read_csv(bag_folder + 'current_pose.csv')
current_pose = dict()
for col in wp:
    if "position" in col:
        current_pose[col[-1]] = wp[col]


# create lists for matplotlib
xs = []
ys = []
for col, wp in base_waypoints.items():
    xs.append(wp['x'])
    ys.append(wp['y'])

final_xs = final_waypoints['x']
final_ys = final_waypoints['y']

current_xs = current_pose['x']
current_ys = current_pose['y']


# /traffic_waypoint topic
wp = pd.read_csv(bag_folder + 'traffic_waypoint.csv')
traffic_xs = []
traffic_ys = []
for col in wp:
    if "data" in col:
        for wp_idx in list(wp[col]):
            if wp_idx != -1:
                traffic_xs.append(xs[wp_idx])
                traffic_ys.append(ys[wp_idx])


# plot
plt.rcParams["figure.figsize"] = [16, 16]
p1 = plt.plot([xs[0]], [ys[0]], 'ko', ms=10.0)
p2 = plt.plot(xs, ys, 'go', ms=5.)
p3 = plt.plot(final_xs, final_ys, 'c', ms=5.0)
p4 = plt.plot(current_xs, current_ys, 'r', lw=5.0)
p5 = plt.plot([current_xs[0]], [current_ys[0]], 'ko', ms=15.0)
p6 = plt.plot(traffic_xs, traffic_ys, 'rx', ms=10.0)
# p6 = plt.plot(dbw_enabled_x, dbw_enabled_y, 'gx', ms=20.0, mew=10.0)
# p7 = plt.plot(dbw_disabled_x, dbw_disabled_y, 'rx', ms=20.0, mew=10.0)
plt.xlabel("X", fontsize=10)
plt.ylabel("Y", fontsize=10)
# plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0]), ('first base_waypoint', 'base_waypoints', 'final_waypoints', 'current position', 'first current position', 'dbw enabled', 'dbw disabled'), loc=0)
plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0], p6[0]), ('first base', 'base_waypoints', 'final_waypoints', 'current position', 'first current', 'traffic_waypoint'), loc=0)
plt.show()
