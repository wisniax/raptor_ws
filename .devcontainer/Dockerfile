# Use the official image as a parent image
FROM osrf/ros:noetic-desktop-full

# Dockerfile for ROS-NOETIC

# ENV DEBIAN_FRONTEND noninteractive

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano
# gedit vim neovim firefox

# Random Stuff
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN ln -sf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime

RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
# RUN apt install -y ros-noetic-ros-canopen

# Environment setup
RUN mkdir /root/raptor_ws