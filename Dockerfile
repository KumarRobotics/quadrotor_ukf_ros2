FROM osrf/ros:humble-desktop  

LABEL maintainer="kashg@seas.upenn.edu"

RUN apt-get update && apt-get install -y \
    vim 
RUN mkdir -p ~/qukf_ws/src 
RUN cd ~/qukf_ws/src &&  . /opt/ros/humble/setup.sh && ros2 pkg create --build-type ament_cmake quadrotor_ukf_ros2
