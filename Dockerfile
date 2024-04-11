# Start with the Ubuntu 20.04 LTS base image
FROM ubuntu:20.04

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic \
    BASH_ENV=~/.bashrc \
    LD_LIBRARY_PATH=/usr/local/lib

# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install packages necessary for ROS, OpenCV, python3-pip for rosdep and cleanup in a single RUN to reduce layers
RUN apt-get update && apt-get install -y \
        software-properties-common \
        lsb-release \
        wget \
        curl \
        build-essential \
        cmake \
        git \
        libgtk2.0-dev \
        pkg-config \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        python3-dev \
        python3-numpy \
        python3-pip \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libdc1394-22-dev \
        gdb \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update && apt-get install -y ros-noetic-desktop-full \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    # Install rosdep after installing ROS to ensure rosdep command is available
    && pip3 install -U rosdep \
    && rosdep init && rosdep update \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    # Cleanup to keep the image clean and compact
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

WORKDIR /root

# Clone OpenCV and OpenCV Contrib repositories
RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git

# Create and switch to the build directory
WORKDIR /root/opencv/build

# Configure the build with CMake
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D INSTALL_C_EXAMPLES=ON \
          -D INSTALL_PYTHON_EXAMPLES=ON \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -D BUILD_EXAMPLES=ON .. && \
    make -j$(nproc) && \
    make install

# Set the working directory to root
WORKDIR /root

# Setup the entry point to launch the ROS environment
CMD ["bash"]