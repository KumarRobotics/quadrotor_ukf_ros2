FROM osrf/ros:humble-desktop  
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all


LABEL maintainer="kashg@seas.upenn.edu"

RUN apt-get update && apt-get install -y \
    vim 
RUN mkdir -p ~/qukf_ws/src/quadrotor_ukf_ros2/
WORKDIR /root/qukf_ws/src/quadrotor_ukf_ros2

COPY ./src ./src
COPY ./include ./include
COPY ./CMakeLists.txt ./
COPY ./package.xml ./
COPY ./bags ../../../

WORKDIR /root/qukf_ws/

# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
# RUN . /opt/ros/humble/setup.sh && colcon build

