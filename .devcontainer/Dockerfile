# Use the official image as a parent image
FROM osrf/ros:noetic-desktop-full

####################################################################################################
# Dockerfile for ROS-NOETIC
# CONFIGURATION
#
# Create user 'name' with sudo privileges
ARG USERNAME='vscode'
ARG PASSWORD='vscode'

####################################################################################################

ENV DEBIAN_FRONTEND noninteractive

# Update the system
RUN apt-get update 

# Install OpenSSH Server
RUN apt-get install -y git gdb net-tools nano curl wget python3 python3-pip apt-utils gedit nano vim neovim net-tools

# Random Stuff
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN ln -sf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime

RUN useradd -m -s /bin/bash ${USERNAME}
RUN echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN usermod -aG sudo ${USERNAME}
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc
RUN echo "source /opt/ros/noetic/setup.sh" >> /home/${USERNAME}/.bashrc
RUN apt install -y ros-noetic-ros-canopen

# USER $USERNAME
# Environment setup
RUN mkdir /home/${USERNAME}/raptor_ws