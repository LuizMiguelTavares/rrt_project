FROM dev-container:latest

## Enable if you have an NVIDIA GPU and want to use it in the container 
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ARG USERNAME=dev_container
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-slam-gmapping \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Create the directory as root
RUN mkdir -p /home/$USERNAME/catkin_ws/src \
    && chown -R $USERNAME:$USER_GID /home/$USERNAME

# Switch from root to user
USER $USERNAME

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
WORKDIR /home/$USERNAME/catkin_ws/src
