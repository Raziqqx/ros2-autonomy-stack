# One image for building, testing and running the stack headless.
# ros-base (not desktop): no GUI/Gazebo, ~1.3 GB, and it already ships colcon,
# ament_cmake_gtest, launch_testing, rosbag2 and the message packages this
# stack uses. The only addition is BehaviorTree.CPP v4 from the ROS apt repo
# (~30 MB with its ZeroMQ deps), in its own layer so source edits reuse it.
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY src ./src
# A few parallel workers keep peak RAM modest on small machines; the colcon
# log directory is dropped to keep the layer small.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --parallel-workers 4 --event-handlers console_cohesion+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    && rm -rf /ws/log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
