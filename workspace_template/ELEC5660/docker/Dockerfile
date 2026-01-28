FROM ubuntu:20.04

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Beijing \
    PATH=$PATH:/opt/VirtualGL/bin

# Arguments for build
ARG ROS_VERSION=noetic

# Combine System, Dependencies, ROS, and Desktop Installation to reduce layers
RUN sed -i "s@http://.*archive.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    # Basic Build Tools & Utilities
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    unzip \
    zip \
    htop \
    xterm \
    net-tools \
    lsb-release \
    gnupg2 \
    ca-certificates \
    # Development Libraries
    libgtk2.0-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpcap-dev \
    binutils-dev \
    libdw-dev \
    libdwarf-dev \
    gdb \
    # Python
    python3-pip \
    python3-dev \
    # Desktop Environment & VNC dependencies
    dbus-x11 \
    sudo \
    novnc \
    x11vnc \
    xvfb \
    supervisor \
    xfce4 \
    xfce4-goodies \
    ssh \
    terminator \
    nautilus \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-xserver-utils && \
    # Setup Official ROS Source
    sh -c 'echo "deb http://repo.huaweicloud.com/ros/ubuntu/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    # ROS Packages
    ros-${ROS_VERSION}-ros-base \
    ros-${ROS_VERSION}-nav-msgs \
    ros-${ROS_VERSION}-sensor-msgs \
    ros-${ROS_VERSION}-cv-bridge \
    ros-${ROS_VERSION}-rviz \
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-image-transport-plugins \
    ros-${ROS_VERSION}-rqt-gui \
    ros-${ROS_VERSION}-rqt-common-plugins \
    ros-${ROS_VERSION}-catkin \
    ros-${ROS_VERSION}-tf-conversions \
    ros-${ROS_VERSION}-gazebo-ros-pkgs \
    ros-${ROS_VERSION}-gazebo-ros-control \
    ros-${ROS_VERSION}-turtlebot3-description \
    ros-${ROS_VERSION}-teleop-twist-keyboard \
    ros-${ROS_VERSION}-velodyne-simulator \
    gazebo11 \
    # ROS Tools
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools && \
    # ROS Initialization
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc && \
    # Clean up apt caches
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Firefox
RUN apt-get update && \
    apt-get install -y firefox && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install VirtualGL
RUN wget https://github.com/VirtualGL/virtualgl/releases/download/3.1/virtualgl_3.1_amd64.deb && \
    apt-get update && \
    apt-get install -y ./virtualgl_3.1_amd64.deb && \
    rm virtualgl_3.1_amd64.deb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# Install Ceres Solver
# Download, build, install, and clean up in one layer
RUN wget -qO - https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz | tar zx -C /tmp && \
    cd /tmp/ceres-solver-1.14.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/ceres-solver-1.14.0

# Install Python packages
RUN python3 -m pip install --no-cache-dir \
    "evo==1.31.1" \
    "nicegui==2.24.2" \
    "plotly==6.5.2" \
    "numpy==1.24.4" \
    "packaging==26.0" \
    "Pygments>=2.15.1,<3" \
    "python-dateutil>=2.8.2"

# Install Aruco
COPY aruco-3.1.15 /tmp/aruco-3.1.15
RUN /bin/bash -c "cd /tmp/aruco-3.1.15 \
    && mkdir -p build \
    && cd build \
    && source /opt/ros/${ROS_VERSION}/setup.bash \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/aruco-3.1.15"

# Install noVNC
COPY novnc.zip /tmp/novnc.zip
RUN unzip -o /tmp/novnc.zip -d /usr/share && rm /tmp/novnc.zip

# Copy system files and configure
COPY . /system
RUN chmod +x /system/conf.d/websockify.sh /system/supervisor.sh

# Write environment variables in /etc/profile.d/ros.sh
RUN touch /etc/profile.d/ros.sh && \
    echo 'export DISPLAY=:10' >> /etc/profile.d/ros.sh && \
    echo 'export VGL_DISPLAY=:10' >> /etc/profile.d/ros.sh && \
    echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> /etc/profile.d/ros.sh