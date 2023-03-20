FROM osrf/ros:kinetic-desktop-full
#install depends libs and then install ceres-soplver from source
RUN set -x; buildDeps='libgoogle-glog-dev libgflags-dev libatlas-base-dev  libsuitesparse-dev wget vim' \
    && apt-get update \
    && apt-get install -y $buildDeps \
# download ceres-solver source code and then compile
    && wget ceres-solver.org/ceres-solver-1.14.0.tar.gz \
    && mkdir -p /home/3rd_party/\
    && tar -xzf ceres-solver-1.14.0.tar.gz -C /home/3rd_party/\
    && mkdir -p /home/3rd_party/ceres-solver-1.14.0-build \
    && cd /home/3rd_party/ceres-solver-1.14.0-build \
    && cmake /home/3rd_party/ceres-solver-1.14.0 \
    && make \
    && make install \
    && rm /ceres-solver-1.14.0.tar.gz

# nvida parameters 
# LABEL com.nvidia.volumes.needed="nvidia_driver"
# ENV PATH /usr/local/nvidia/bin:${PATH}
# ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}pkg-config \

# ENV NVIDIA_VISIBLE_DEVICES \
# ${NVIDIA_VISIBLE_DEVICES:-all}

# ENV NVIDIA_DRIVER_CAPABILITIES \
# ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# RUN apt-get update && \
# apt-get install -y \
# build-essential \
# libgl1-mesa-dev \
# libglew-dev \
# libsdl2-dev \
# libsdl2-image-dev \
# libglm-dev \
# libfreetype6-dev \
# libglfw3-dev \
# libglfw3 \
# libglu1-mesa-dev \
# freeglut3-dev \