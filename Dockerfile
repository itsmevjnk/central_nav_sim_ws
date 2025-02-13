# ROS Humble with CycloneDDS
FROM ros:humble-ros-base-jammy AS base
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get install -y ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# our stuff
FROM base AS overlay

# copy files over
RUN mkdir -p /cnav_ws
COPY . /cnav_ws

# install dependencies
WORKDIR /cnav_ws
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && colcon build --symlink-install



