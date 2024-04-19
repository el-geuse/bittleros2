FROM althack/ros2:humble-cuda-gazebo-nvidia


ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-serial curl \
   ros-humble-v4l2-camera ros-humble-image-transport-plugins \
   && curl -s https://install.husarnet.com/install.sh | bash \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Necessary Python packages
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Linter configuration settings
# ARG WORKSPACE
COPY ${WORKSPACE}/ament_flake8.ini /opt/ros/humble/lib/python3.10/site-packages/ament_flake8/configuration/ament_flake8.ini

# Set up auto-source of workspace for ros user
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Set up the initial script
COPY ${WORKSPACE}/initial-params.sh /initial-params.sh

# Changing initial script permissions and running
RUN chmod +x /initial-params.sh
RUN /initial-params.sh

ENV DEBIAN_FRONTEND=dialog