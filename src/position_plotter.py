#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import time
import numpy as np
import signal
import csv
import matplotlib.pyplot as plt
def read_csv_data(filename):
    try:
        values =[]
        buffer = None
        with open(filename) as csvfile:
            buffer = csv.reader(csvfile, delimiter=',')
            for row in buffer:
                values = np.array(row)
    except csv.Error:
        print('Error reading {filename}')
    values = map (float, values)
    return values
x =[]
y =[]
x.append(0)
y.append(0)
velocity_x = read_csv_data("/home/houman/catkin_ws/src/trajectory_visualizer/sample_data/quadrotor/xdot.csv")
velocity_y = read_csv_data("/home/houman/catkin_ws/src/trajectory_visualizer/sample_data/quadrotor/ydot.csv")

for i in range (1,np.size(velocity_x)):
    x.append(x[i-1] + velocity_x[i]*0.02)
    y.append(y[i-1] + velocity_y[i]*0.02)

fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.plot(x,y,'ro')
markerprops = dict(marker='o', markersize=5, markeredgecolor='blue')
plt.grid(True)
plt.axis('equal')

plt.show()