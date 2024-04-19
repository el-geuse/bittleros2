#!/bin/bash
# Starter file for ROS / Colcon Docker containers
 
# Source ROS 2
# source /opt/ros/${ROS_DISTRO}/setup.bash
# (Already sourced in base docker image)
echo "export ROS_DOMAIN_ID=42" >> /home/ros/.bashrc

# Required Husarnet ROS 2 variables
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/clyderos2/src/clyde_teleop/config/fastdds-ds-client.xml" >> /home/ros/.bashrc
echo "export DISCOVERY_SERVER_PORT=11811" >> /home/ros/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/ros/.bashrc
echo "export ROS_DISCOVERY_SERVER=clydepc:11811" >> /home/ros/.bashrc

# Source Gazebo
echo "source /usr/share/gazebo/setup.bash" >> /home/ros/.bashrc

# Command to start server
echo "alias serverstart='unset ROS_DISCOVERY_SERVER && fastdds discovery -x /workspaces/clyderos2/src/clyde_teleop/config/fastdds-ds-server.xml'" >> /home/ros/.bashrc

# With FastDDS discovery v2, ROS introspection tools do not work, as they require a node to be made,
# which then can't read the restricted topic. Running this command beforehand bypasses that.
echo "alias superclient='export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/clyderos2/src/clyde_teleop/config/super_client_configuration_file.xml && ros2 daemon stop && ros2 daemon start'" >> /home/ros/.bashrc

# This is a temporary solution to ensure that the node has the relevant FastDDS priviliges,
# Leaving this out would give a node full permissions after opening a superclient.
# echo "ros2 daemon stop && ros2 daemon start" >> /home/ros/.bashrc
# Commented out while debugging