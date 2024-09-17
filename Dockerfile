FROM osrf/ros:humble-desktop  
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

#ROS-Domain
ENV ROS_DOMAIN_ID=1
LABEL maintainer="kashg@seas.upenn.edu"

RUN mkdir -p /run/user/1000
RUN chmod 0700 /run/user/1000

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    iproute2 \
    ros-humble-tf2-eigen
RUN mkdir -p ~/qukf_ws/src/quadrotor_ukf_ros2/
WORKDIR /root/qukf_ws/src/quadrotor_ukf_ros2

# ROS2 Specific
COPY ./src ./src
COPY ./include ./include
COPY ./launch ./launch
COPY ./CMakeLists.txt ./
COPY ./package.xml ./

# Dev Specific
COPY ./bags ../../../
COPY ./.bashrc ../../../


WORKDIR /root/qukf_ws/
#RUN . /opt/ros/humble/setup.sh && colcon build


