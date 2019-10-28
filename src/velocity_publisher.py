#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry 
from std_msgs.msg import Header
import time
import numpy as np
import signal
import csv
    
robot_odom_x = 0
robot_odom_y = 0

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

def position_reader(data):
    global robot_odom_x
    global robot_odom_y
    robot_odom_x = data.pose.pose.position.x
    robot_odom_y = data.pose.pose.position.y

if __name__=="__main__":

    rospy.init_node('velocity_publisher', anonymous=True, disable_signals=True)
    robot_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    position_subscriber = rospy.Subscriber("odom", Odometry, position_reader)
    
    
    robot_x_path = rospy.get_param('~x_path')
    robot_y_path = rospy.get_param('~y_path')
    robot_x_velocity_path = rospy.get_param('~vx_path')
    robot_y_velocity_path = rospy.get_param('~vy_path')
    duration = rospy.get_param('~duration')
    
    robot_x = read_csv_data(robot_x_path)
    robot_y = read_csv_data(robot_y_path)
    
    robot_vx = read_csv_data(robot_x_velocity_path)
    robot_vy = read_csv_data(robot_y_velocity_path)
    
    dt = float(duration)/np.size(robot_vx)
    counter = 0

    ## Defining paths for themm to get visualized in Rviz
    robot_trajectory = Path()

  
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
    
    actual_trajectory_final_x = robot_x[-1]
    actual_trajectory_final_y = robot_y[-1]
    
    twist = Twist()
    robot_cmd_vel.publish(twist)
    
    rospy.sleep(1)
    path_publisher.publish(robot_trajectory)

    rate = rospy.Rate(1/dt)
    start = time.time() 
    try:
        while (counter < np.size(robot_vx)):
            twist.linear.x = robot_vx[counter]
            twist.linear.y = robot_vy[counter]
            twist.angular.z = 0
            counter+=1
            robot_cmd_vel.publish(twist)
            rate.sleep()
    except KeyboardInterrupt:
            print ("ctrl + c was pressed! Program terminated!")
    
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0
    robot_cmd_vel.publish(twist)
    end = time.time()
    
    print ("Execution time: " + str(end-start))
    print ("The trajectory final position is: x = {} and y = {}".format(actual_trajectory_final_x, actual_trajectory_final_y))
    print ("The robot reached position: x = {} and y = {}".format(robot_odom_x, robot_odom_y))
    exit()