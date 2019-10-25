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
robot_x = read_text("robot_x_pose.txt")
robot_y = read_text("robot_y_pose.txt")

 ## Defininf paths for themm to get visualized in Rviz
robot_trajectory = Path()
dt = 30/242

robot_vx = []
robot_vy = []

## Filling the coordinate vectors
for i in range (np.size(robot_x)-1):
     robot_vx.append((robot_x[i+1] - robot_x[i])/dt)
     robot_vy.append((robot_y[i+1] - robot_y[i])/dt)

rospy.init_node('velocity_publisher')
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
    
while (not rospy.is_shutdown()):  
    path_publisher.publish(robot_trajectory)
    
    if (counter==0):
        for i in range (np.size(robot_vx)):
            t_start = time.time()
            twist = Twist()
            twist.linear.x = robot_vx[i]
            twist.linear.y = robot_vy[i]
            twist.linear.z = 0
            robot_cmd_vel.publish(twist)
            t_end  = time.time()
            time_difference = t_end - t_start
            rospy.sleep(dt - time_difference)
            counter+=1
            path_publisher.publish(robot_trajectory)
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        robot_cmd_vel.publish(twist)