# main_NAV_ws
official ROS workspace to group all the packages af the NAV subsystem


## dependencies

ROS-Melodic !!!
```
sudo apt-get update
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install python-catkin-tools
```

## setup

at the root of the repo :
```
catkin init
catkin build
```

## run
* To visualize the rover in rViz run: 
```bash
roslaunch rover_description display.launch 
```
* To start Gazebo, ros_control and spawn the rover run:
```bash
roslaunch rover_description gazebo.launch
```
In rViz, the robot model and the reference frame should be manually selected

Once Gazebo is running, a controller can be launched with the following 2 nodes:
```bash
rosrun wheels_control twist_to_wheels.py
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

