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
```

---

# turtle_walker

Contains a /cmd_vel publisher that uses the /scan topic to avoid obstacles in front of the robot.
```bash
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