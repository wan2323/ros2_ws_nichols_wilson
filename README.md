# ros2_ws_nichols_wilson

### ROBE 313 – West Virginia University  
**Author:** Wilson Nichols
**Instructor:** Dr. Giacomo Marani  
**Semester:** Fall 2025  

This project implements a ROS 2 simulation integrating C++ and Python nodes to control a mobile robot moving in a 2 m × 2 m square trajectory.  

The workspace is organized into four modular packages:

`custom_interfaces` 
--> C++
--> Defines the custom service `ResetPosition.srv` used to reset odometry.
`robot_simulator_cpp`
--> C++
--> Implements the `odometry_node`, which subscribes to `/cmd_vel`, publishes tf transforms, and hosts the ResetPosition service.
`robot_simulator_py` 
--> Python
--> Implements the `controller_node`, which publishes `/cmd_vel` commands and uses tf2 to follow a square path.
`robot_bringup` 
--> Python 
--> Integrates all packages, including URDF, RViz configuration, and launch files. |

System Requirements
- ROS 2 Humble Hawksbill  
- Ubuntu 22.04 LTS  

Setup Instructions

1. Create and Initialize the Workspace

mkdir -p ~/ros2_ws_nichols_wilson/src
cd ~/ros2_ws_nichols_wilson/src

git clone https://github.com/wan2323/ros2_ws_nichols_wilson.git

cd ~/ros2_ws_nichols
colcon build --symlink-install
source install/setup.bash

2. Run Simulation

ros2 launch robot_bringup robot_simulation.launch.py



