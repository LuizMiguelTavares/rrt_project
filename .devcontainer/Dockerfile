FROM dev-container:latest

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