# ROS Distro
ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO

# Set working directory
RUN mkdir /ws
WORKDIR /ws

RUN sudo apt update && sudo apt install -y -q --no-install-recommends ros-humble-geographic-msgs ros-humble-ros2-socketcan libboost-all-dev

# Copy repo into the container
COPY . /ws/src

# Source ROS and build package
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-up-to brake_temperature_controller_node_cpp gear_shift_controller_node_cpp longitudinal_controller_node_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. \
    /ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]