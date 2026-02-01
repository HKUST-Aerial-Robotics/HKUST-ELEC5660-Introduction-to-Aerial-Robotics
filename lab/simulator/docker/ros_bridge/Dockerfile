FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 humble dependencies
RUN apt-get update -y && \
    apt-get install -y \
    curl gnupg lsb-release apt-utils && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -y && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-rosdep2 \
    python3-setuptools \
    locales \
    wget && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install -U colcon-common-extensions vcstool empy==3.3.4

# Build ROS2 humble
RUN mkdir -p /ros2_humble/src && \
    cd /ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    apt-get update && \
    apt upgrade -y && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    colcon build --symlink-install && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS1 noetic
RUN rm /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt remove -y \
    python3-catkin-pkg python3-catkin-pkg-modules &&\
    apt install -y \
    ros-core-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Build ros1_bridge
RUN mkdir -p /ros1_bridge/src && \
    cd /ros1_bridge/src && \
    git clone https://github.com/ros2/ros1_bridge && \
    cd ros1_bridge && \
    # https://github.com/ros2/ros1_bridge/issues/391
    git checkout b9f1739fd84fc877a8ec6e5c416b65aa2d782f89 && \
    cd /ros1_bridge && \
    bash -c ". /ros2_humble/install/local_setup.bash && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

ENV ROS_MASTER_URI=http://127.0.0.1:11311
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/dds_profile.xml
COPY entrypoint.sh /entrypoint.sh
COPY dds_profile.xml /dds_profile.xml
ENTRYPOINT [ "/entrypoint.sh" ]
