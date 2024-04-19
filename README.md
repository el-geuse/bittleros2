# clyderos2
## EM501 Project
### Group C

Master's Thesis group project working on creating a quadruped capable of use in elderly care, specifically fall detection and autonomous supervision. Built using ROS 2, with a Raspberry Pi Zero 2W attached to the quadruped frame, issuing data to the master pc.

## Folder Breakdown
Packages broken down in a pretty standard way:
1. **`.devcontainer`**: Configuration for the development container environment.
2. **`clyde_audio`** : Source code for wakeword and command detection.
3. **`clyde_bringup`** : Contains launch files used in all the other packages.
4. **`clyde_control`** : Package for body motion planning and gait generation.
5. **`clyde_description`** : Files needed for representation of Clyde's model.
6. **`clyde_driver`** : Code needed to communicate with Bittle's serial interface.
     - Contains the code to be run on the raspberry pi.
     - Contains publishers for audio, vision and pose.
7. **`clyde_msgs`** : Package detailing all messages defined within the system.
8. **`clyde_nav`** : Code used for autonomous navigation.
9. **`clyde_teleop`** : Code required for teleoperation of clyde (controller, configuration files).
10. **`clyde_vision`** : Source code for human and fall detection.

## Building
To build Clyde, ensure that ROS 2 is installed and run the following commands:
```bash
source /opt/ros/<ros2-distro>/setup.bash
mkdir -p ~/clyde_ws/src
cd ~/clyde_ws
colcon build --symlink-install
```
## Dependencies
For dependencies of both systems ensure the following are installed:
- Docker
- nvidia-container-toolkit
- VSCode
- Devcontainers for VSCode

Upon opening the project in VSCode, the option to open in a devcontainer will show up. Build the container then proceed.

## Launching
To launch the Clyde system, use the included launch files in **`clyde_bringup`** while inside the docker container:
On the PC:
```bash
source ~/clyde_ws/install/setup.bash
ros2 launch clyde_bringup clyde_full.launch.py
```
In a new terminal:
```bash
serverstart
```
On the Pi:
```bash
source ~/clyde_ws/install/setup.bash
ros2 launch clyde_bringup clyde_pi_full.launch.py
```

## Contributions
We welcome contributions via pull requests. For major changes, please open an issue first to discuss what you would like to change. Ensure to update tests as appropriate.
Contact [@el-geuse](https://github.com/el-geuse) for more information or clarification.
