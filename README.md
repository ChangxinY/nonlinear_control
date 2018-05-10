# Trajectory Planning and Control for A Nonholonomic Robot Among Obstacles
Course project for ME.530.678 Nonlinear Control and Planning in Robotics (2018) <br />

## Description
This project uses RRT* based algorithm to do the path planning and further develops path into a trajectory, and uses 
feedback linearization to do the trajectory tracking. Simulation is done in Gazebo. This package reads the predesighed 
trajectory in a txt file and simulates the trajectory tracking.  <br />

to run the simulation
```
roslaunch robot gazebo_launch.launch
```
to start the simulation
```
roslaunch robot trajectory_tracking.launch
```
to visulize the odometry path of the robot in RVIZ
```
roslaunch robot rviz_launch.launch
```
![alt text](https://github.com/ChangxinY/nonlinear_control/blob/master/images/simulation_setup.png)
