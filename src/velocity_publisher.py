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

if __name__=="__main__":

    rospy.init_node('velocity_publisher', anonymous=True, disable_signals=True)
    robot_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    robot_x_path = rospy.get_param('~x_path')
    robot_y_path = rospy.get_param('~y_path')
    duration = rospy.get_param('~duration')
    robot_x = read_csv_data(robot_x_path)
    robot_y = read_csv_data(robot_y_path)

    dt = float(duration)/np.size(robot_x)
    robot_vx = []
    robot_vy = []
    counter = 0

    ## Defining paths for themm to get visualized in Rviz
    robot_trajectory = Path()

    ## Filling the coordinate vectors
    for i in range (np.size(robot_x)-1):
        robot_vx.append((robot_x[i+1] - robot_x[i])/dt)
        robot_vy.append((robot_y[i+1] - robot_y[i])/dt)
  
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
            print ("ctrl + c was pressed! Program terminated!")
    
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    robot_cmd_vel.publish(twist)
    exit()