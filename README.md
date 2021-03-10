# main_NAV_ws
official ROS workspace to group all the packages af the NAV subsystem


## dependencies

ROS-Melodic !!!
```
sudo apt-get update
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-roslint ros-melodic-robot-state-publisher ros-melodic-eigen-conversions ros-melodic-pcl-ros ros-melodic-tf-conversions 
sudo apt-get install python-catkin-tools
sudo apt install ros-melodic-joint-state-publisher-gui
sudo apt-get install libeigen3-dev
sudo apt-get install ros-melodic-grid-map
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-ar-track-alvar
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

* To start the path planner ( u can visualise path on Rviz and use 2d-target as target )
```bash
roslaunch mybot_navigation nav.launch
rosrun publishers map_differentiator
```

