#!/usr/bin/env python
from __future__ import division
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
import numpy as np
import signal

############
def read_text (text_file):
    text = []
    with open(text_file) as f:
        for i in f.read().split(' '):
            text.append(i)
    del text[-1]
    text = map (float, text)
    return text
#############


counter = 0
robot_x = np.linspace(0,5,10)
robot_y = np.linspace(0,5,10)

 ## Defininf paths for themm to get visualized in Rviz
robot_trajectory = Path()
dt = 5/10

robot_vx = []
robot_vy = []

## Filling the coordinate vectors
for i in range (np.size(robot_x)-1):
     robot_vx.append((robot_x[i+1] - robot_x[i])/dt)
     robot_vy.append((robot_y[i+1] - robot_y[i])/dt)

rospy.init_node('velocity_publisher', anonymous=True, disable_signals=True)
robot_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
path_publisher = rospy.Publisher('path', Path, queue_size=10)
for i in range (np.size(robot_x)):
    robot_pose = PoseStamped()
    header  = Header()
    header.frame_id = "odom"
    robot_pose.pose.position.x = robot_x[i]
    robot_pose.pose.position.y = robot_y[i]
    robot_pose.pose.orientation.x = 0
    robot_pose.pose.orientation.y = 0
    robot_pose.pose.orientation.z = 0
    robot_pose.pose.orientation.w = 1
    robot_pose.header = header
    robot_trajectory.header.frame_id = "odom"
    
    robot_trajectory.poses.append(robot_pose)

twist = Twist()
robot_cmd_vel.publish(twist)
rospy.sleep(1)
try:    
    while (counter < np.size(robot_vx)):  
        path_publisher.publish(robot_trajectory)
        t_start = time.time()
        twist.linear.x = robot_vx[counter]
        twist.linear.y = robot_vy[counter]
        twist.angular.z = 0
        counter+=1
        robot_cmd_vel.publish(twist)
        t_end  = time.time()
        time_difference = t_end - t_start
        rospy.sleep(dt - time_difference)
except KeyboardInterrupt:
        print "Interrupt happened! Program terminated!"
twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
robot_cmd_vel.publish(twist)