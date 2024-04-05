FROM dev-container:2.0
RUN apt update \
    && apt install -y ros-noetic-ros-control \
	ros-noetic-ros-controllers

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
