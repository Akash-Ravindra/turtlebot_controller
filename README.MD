# Turtle Bot 3 Controller

The package contains a controller that uses the /scan topic to avoid obstacle.

---

# Packages

1. [Turtle Walker](./turtle_walker/)

---

# Environment

Using ROS2 Humble docker image that can be found here `osrf/ros:humble-desktop-full`

```bash
#Pull the docker image
docker pull osrf/ros:humble-desktop-full
# Run ROS 2 container
rocker osrf/ros:humble-desktop-full bash

# inside ROS 2 container, install terminator
apt update;
apt -y install terminator
apt -y install ros-humble-gazebo-ros-pkgs 
apt -y install ros-humble-turtlebot3*
apt clean all

#Use another terminal to save the docker image
CONTAINER_ID=$(docker ps --format {{.ID}})
echo $CONTAINER_ID

# Save the Docker image snapshot 
docker commit $CONTAINER_ID my-docker2-humble
```
Now the env is set up at any time use the following command to start the container
```bash 
# no graphic acceleration
rocker --x11 --user --home --privileged my-docker2-humble terminator
```

Create a ws and download the package
```bash
mkdir -p ~/ws/src
cd ~/ws/src/
git clone https://github.com/Akash-Ravindra/turtlebot_controller.git
cd ..
colcon build
```

---

# turtle_walker

Contains a /cmd_vel publisher that uses the /scan topic to avoid obstacles in front of the robot.
```bash
cd ~/ws/
# Run the executable of the controller 
ros2 run turtle_walker controller
```

 To launch both the controller and the gazebo environment in a custom world

 ```bash
 ros2 launch turtle_walker turtulebot3_gazebo_controller.launch.py
 ```
 To enable recording
 ```bash
 ros2 launch turtle_walker turtulebot3_gazebo_controller.launch.py record_enabled:='True'
 ```
 To replay a rosbag along with the simulation to replay the simulation(read and publish only the /cmd_vel topic that was recorded)
 ```bash
 # The recording will end abruptly
 ros2 launch turtle_walker turtulebot3_gazebo_controller.launch.py replay_only:='True' bag_file:='./src/turtle_walker/results/recorder1670134485.9357378.bag/'
 ```