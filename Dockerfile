FROM osrf/ros:humble-desktop  

LABEL maintainer="kashg@seas.upenn.edu"

RUN apt-get update && apt-get install -y \
    vim 
RUN mkdir -p ~/qukf_ws/src/quadrotor_ukf_ros2/
WORKDIR /root/qukf_ws/src/quadrotor_ukf_ros2

COPY ./src ./src
COPY ./include ./include
COPY ./CMakeLists.txt ./
COPY ./package.xml ./

WORKDIR /root/qukf_ws/
# RUN . /opt/ros/humble/setup.sh && colcon build

