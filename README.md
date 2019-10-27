# trajectory_visualizer

This package is used for visualizing a robot trajectory in Rviz.

The robot is omnidirectional and accepts linear.x, linear.y and angular.z velocity values in form a of Twist message.
The user should provide both the waypoints for the trajetory and correpsonding velocities to the visualizer in form of two CSV files. 
The user can see whether the robot would follow the trajectory using the corresponding velocities generated.

# Published and subscribed topics
/odom (nav_msgs/Odometry)
/cmd_vel (geometry_msgs/Twist)
