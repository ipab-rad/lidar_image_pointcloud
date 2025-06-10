FROM ros:humble-ros-base-jammy AS base

# Install key dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        curl \
        ros-"$ROS_DISTRO"-ament-cmake-clang-format \
        ros-"$ROS_DISTRO"-image-transport \
        ros-"$ROS_DISTRO"-image-transport-plugins \
        ros-"$ROS_DISTRO"-sensor-msgs \
        ros-"$ROS_DISTRO"-std-msgs \
        ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./cyclone_dds.xml $ROS_WS/

# Configure Cyclone cfg file
ENV CYCLONEDDS_URI=file://${ROS_WS}/cyclone_dds.xml

# Enable ROS log colorised output
ENV RCUTILS_COLORIZED_OUTPUT=1

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Import sensor code from repos
COPY lidar_image_pointcloud $ROS_WS/src/lidar_image_pointcloud

# Source ROS setup for dependencies and build our code
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# -----------------------------------------------------------------------

FROM base AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Add sourcing local workspace command to bashrc for convenience when running interactively
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    # Add colcon build alias for convenience
   && echo 'alias colcon_build="colcon build --symlink-install --cmake-args \
   -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"' >> /root/.bashrc

# Enter bash for development
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base AS runtime

# Copy artifacts/binaries from prebuilt
COPY --from=prebuilt $ROS_WS/install $ROS_WS/install

# Add command to docker entrypoint to source newly compiled
#   code when running docker container
RUN sed --in-place --expression \
        "\$isource \"$ROS_WS/install/setup.bash\" " \
        /ros_entrypoint.sh

# Launch ros package
CMD ["ros2", "launch", "lidar_image_pointcloud", "lidar_image_pointcloud.launch.py"]
