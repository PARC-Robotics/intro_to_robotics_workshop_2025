# PARC 2025 Introduction to Robotics Workshop Repository

## Instructions:

### Pull changes from PARC Engineers League repository

A file was added to the PARC 2025 Engineers League repository which is needed for this workshop. Navigate to your workspace then pull the repository
change,

```
cd ~/ros2_ws/src
git pull origin main

```

### Clone the Workshop repository

Place this in the same workspace as the original PARC Engineers League repository

```
cd ~/ros2_ws/src
git clone https://github.com/PARC-Robotics/intro_to_robotics_workshop_2025.git

```

### Compile

```
cd ~/ros2_ws
colcon build
```

### Run

In a terminal, run the following commands to open Gazebo and RViz

```
source ~/ros2_ws/install/setup.bash
ros2 launch intro_to_robotics_workshop_2025 workshop_launch.py

```

In a new terminal run this command to move the robot until it is 1m away from inserted the block

```
ros2 run intro_to_robotics_workshop_2025 workshop_basic_node

```
