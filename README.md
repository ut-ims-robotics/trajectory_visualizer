# trajectory_visualizer

##### This package is used for visualizing a robot trajectory in Rviz.

* The robot is omnidirectional and accepts linear.x, linear.y, and angular.z velocity values in form of a Twist message.
* The user should provide waypoints for the trajetory as well as the corresponding velocity values to the velocity_publisher node in two seperate CSV files.
* The user can see whether and how the robot would follow the trajectory using the corresponding velocities generated.
# Installing dependanceis
``` 
sudo apt-get install ros-kinetic-moveit-visual-tools
 ```
# Published and subscribed topics
* ~/odom (nav_msgs/Odometry)
* ~/cmd_vel (geometry_msgs/Twist)
# Parameters
* ~ x_path (default=/sample_data/x.csv)
* ~ y_path (default=/sample_data/y.csv)
* ~ vx_path (default=/sample_data/xdot.csv)
* ~ vy_path (default=/sample_data/ydot.csv)
* ~ duration (default=60)
# Example usage

In order to get the visualizer and the velocity publisher running you need to first launch the visualizer launch file and then launch the velocity_publisher.launch in another terminal, which can be done through the commands below:

1. In the first terminal: <br/>
``` roslaunch trajectory_visualizer start_visualizer.launch ```
2. In another terminal: <br />
``` roslaunch trajectory_visualizer velocity_publisher.launch ```
<br/>

The x, y, vx, and vy CSV files are located in sample_data folder of the package and the default duration time is set to 60 seconds. You can change the paths as well as the duration time using the parameters available when you launch the velocity_publisher.launch as the following:<br>

``` roslaunch trajectory_visualizer velocity_publisher.launch x_path:=path_to_x y_path:=path_to_y vx_path:= path_to_vx vy_path:=path_to_vy duration:=duration_of_trajectory_execution ```

