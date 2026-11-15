# One image for building, testing and running the stack headless.
# ros-base (not desktop): no GUI/Gazebo, ~1.3 GB, and it already ships colcon,
# ament_cmake_gtest, launch_testing and the message packages this stack uses,
# so no extra apt layer is needed.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY src ./src
# A few parallel workers keep peak RAM modest on small machines; the colcon
# log directory is dropped to keep the layer small.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --parallel-workers 4 --event-handlers console_cohesion+ \
         --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
    && rm -rf /ws/log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
