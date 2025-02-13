# ROS Humble with CycloneDDS
FROM ros:humble-ros-base-jammy AS dds
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# our stuff
FROM dds AS overlay

# copy workspace files over
RUN mkdir -p /cnav_ws/src
COPY src /cnav_ws/src

# install dependencies
WORKDIR /cnav_ws
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys "rviz2" -y \
    && rm -rf /var/lib/apt/lists/*

# build workspace
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# set up benchmark scripts
RUN mkdir -p /scripts
COPY benchmark_scripts/run_benchmark.py /scripts/run_benchmark.py
COPY benchmark_scripts/run_multiple.py /scripts/run_multiple.py
COPY benchmark_scripts/world_points.csv /scripts/world_points.csv
COPY benchmark_scripts/docker_launch.sh /scripts/docker_launch.sh
CMD ["/scripts/docker_launch.sh"]