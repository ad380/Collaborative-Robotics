import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import rosbag
from bagpy import bagreader

bags = ["../bags/gain_half.bag", "../bags/gain1.bag", "../bags/gain2.bag"]

fig, (x_ax, y_ax) = plt.subplots(1, 2)

ref_length = 1001
first = True

for bag in bags:
    bag = bagreader(bag)

    odom = bag.message_by_topic("/locobot/mobile_base/odom")
    odom = pd.read_csv(odom)
    odom_t = odom['Time']
    odom_x = odom['pose.pose.position.x']
    odom_y = odom['pose.pose.position.y']

    ref = bag.message_by_topic("/locobot/mobile_base/goal_pose")
    ref = pd.read_csv(ref)
    ref_t = ref['Time']
    ref_x = ref['position.x']
    ref_y = ref['position.y']

    start = ref_t[0]
    end = ref_t[ref_length - 1] + 2
    start_idx = np.argmax(odom_t > start)
    end_idx = np.argmax(odom_t > end)

    if first:
        x_ax.plot(ref_t[:ref_length] - start, ref_x[:ref_length])
        y_ax.plot(ref_t[:ref_length] - start, ref_y[:ref_length])
        first = False

    x_ax.plot(odom_t[start_idx:end_idx] - start, odom_x[start_idx:end_idx])
    y_ax.plot(odom_t[start_idx:end_idx] - start, odom_y[start_idx:end_idx])


x_ax.legend(["Reference", "0.5 Gain", "1.0 Gain", "2.0 Gain"])
y_ax.legend(["Reference", "0.5 Gain", "1.0 Gain", "2.0 Gain"])
plt.show()

