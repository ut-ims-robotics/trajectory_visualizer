# trajectory_visualizer

##### This package is used for visualizing a robot trajectory in Rviz.

* The robot is omnidirectional and accepts linear.x, linear.y, and angular.z velocity values in form a of Twist message.
* The user should provide waypoints for the trajetory to the velocity_publisher nodes in terms of x and y coordinates in two seperate CSV files.
* The user can see whether and how the robot would follow the trajectory using the corresponding velocities generated.

# Published and subscribed topics
* ~/odom (nav_msgs/Odometry)
* ~/cmd_vel (geometry_msgs/Twist)
# Parameters
* ~ x_path
* ~ y_path
* ~ duration
# Example usage

In order to get the visualizer and the velocity publisher running you need to first launch the visualizer launch file and then run the velocity_publisher node in another terminal, which can be done through the commands below:

1. In the first terminal: <br/>
``` roslaunch trajectory_visualizer start_visualizer.launch ```
2. In another terminal: <br />
``` rosrun trajectory_visualizer velocity_publisher.py ```
