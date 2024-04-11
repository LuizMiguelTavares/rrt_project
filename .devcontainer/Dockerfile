FROM dev-container:latest

## Enable if you have an NVIDIA GPU and want to use it in the container
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install additional packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    nano \
    gedit \
    libcanberra-gtk-module \
    python3-catkin-tools \
    libeigen3-dev \
    ros-noetic-grid-map \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*