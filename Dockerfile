# ROS Humble with CycloneDDS
FROM ros:humble-ros-base-jammy AS dds
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# should be fine if launched with --net=host and host is also set to use docker0 for CycloneDDS
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="docker0"></NetworkInterface></Interfaces></General></Domain></CycloneDDS>'

# our stuff
FROM dds AS overlay-deps

# copy package.xml files over for dependency installation
RUN mkdir -p /cnav_ws/src/nav2_oneshot_nodes
COPY src/nav2_oneshot_nodes/package.xml /cnav_ws/src/nav2_oneshot_nodes/
RUN mkdir -p /cnav_ws/src/benchmark_tools
COPY src/benchmark_tools/package.xml /cnav_ws/src/benchmark_tools/
RUN mkdir -p /cnav_ws/src/ros2_pose_publisher
COPY src/ros2_pose_publisher/package.xml /cnav_ws/src/ros2_pose_publisher/
RUN mkdir -p /cnav_ws/src/nav2_random_goal
COPY src/nav2_random_goal/package.xml /cnav_ws/src/nav2_random_goal/
RUN mkdir -p /cnav_ws/src/tb3_nav_launch
COPY src/tb3_nav_launch/package.xml /cnav_ws/src/tb3_nav_launch/
RUN mkdir -p /cnav_ws/src/nav2_cancel_stopper
COPY src/nav2_cancel_stopper/package.xml /cnav_ws/src/nav2_cancel_stopper/
RUN mkdir -p /cnav_ws/src/tb3_multi_sim/odom_tf_publisher
COPY src/tb3_multi_sim/odom_tf_publisher/package.xml /cnav_ws/src/tb3_multi_sim/odom_tf_publisher/
RUN mkdir -p /cnav_ws/src/tb3_multi_sim/tb3_domain_bridge
COPY src/tb3_multi_sim/tb3_domain_bridge/package.xml /cnav_ws/src/tb3_multi_sim/tb3_domain_bridge/
RUN mkdir -p /cnav_ws/src/tb3_multi_sim/tb3_multi_launch
COPY src/tb3_multi_sim/tb3_multi_launch/package.xml /cnav_ws/src/tb3_multi_sim/tb3_multi_launch/
RUN mkdir -p /cnav_ws/src/central_nav
COPY src/central_nav/package.xml /cnav_ws/src/central_nav/
RUN mkdir -p /cnav_ws/src/nav2_goal_cancel
COPY src/nav2_goal_cancel/package.xml /cnav_ws/src/nav2_goal_cancel/


# install dependencies
WORKDIR /cnav_ws
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys "rviz2" -y \
    && rm -rf /var/lib/apt/lists/*

FROM overlay-deps AS overlay

# copy the rest
COPY src /cnav_ws/src

# build workspace
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

FROM overlay AS scripts

# set up benchmark scripts
RUN mkdir -p /scripts
COPY benchmark_scripts/run_benchmark.py /scripts/run_benchmark.py
COPY benchmark_scripts/run_multiple.py /scripts/run_multiple.py
COPY benchmark_scripts/world_points.csv /scripts/world_points.csv
COPY benchmark_scripts/docker_launch.sh /scripts/docker_launch.sh

RUN mkdir -p /out
WORKDIR /out
CMD ["/scripts/docker_launch.sh"]