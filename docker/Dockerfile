FROM osrf/ros:noetic-desktop-full AS halepensis

# -- DEPENDENCIES
RUN apt-get update && apt-get install -y \
    python3-catkin-tools python3-osrf-pycommon python-tk python3-pip tmux nano vim wget
RUN pip install scipy

# create catkin workspace
WORKDIR /catkin_ws
RUN catkin init
COPY . /catkin_ws/src/halepensis
# build catkin workspace (excluding moveit from the base image)
WORKDIR /catkin_ws
RUN catkin config --extend /opt/ros/noetic && catkin build
RUN echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc

# -- CLEAN-UP
RUN rm -rf /var/lib/apt/lists/*

# --
WORKDIR /
CMD ["/bin/bash"]
