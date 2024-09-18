# Use the official image as a parent image
FROM osrf/ros:noetic-desktop-full

# Dockerfile for ROS-NOETIC

ARG USERNAME='rex'
ARG PASSWORD='changeme'

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano can-utils

# Random Stuff
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN ln -sf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime

# Create user
RUN useradd -m -s /bin/bash ${USERNAME}
RUN echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN usermod -aG sudo ${USERNAME}

# Colorful prompts ah yee
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

# paho-mqtt libraries (for mqtt_bridge)
WORKDIR /tmp
RUN git clone https://github.com/eclipse/paho.mqtt.cpp
WORKDIR /tmp/paho.mqtt.cpp
RUN git checkout v1.4.0
RUN git submodule init
RUN git submodule update
RUN cmake -Bbuild -H. -DPAHO_WITH_MQTT_C=ON -DPAHO_BUILD_EXAMPLES=ON
RUN sudo cmake --build build/ --target install

# Add important stuff to bashrc
RUN echo "\nexport ROS_MASTER_URI=http://192.168.1.20:11311 \nexport ROS_HOSTNAME=192.168.1.20\n" >> /home/${USERNAME}/.bashrc
RUN echo "\nsource /home/rex/raptor-ws/devel/setup.bash \nsource /opt/ros/noetic/setup.sh\n" >> /home/${USERNAME}/.bashrc
# Add alias vnc to start the vnc lmao
RUN echo "\nalias vnc='sudo /usr/local/share/desktop-init.sh'" >> /home/${USERNAME}/.bashrc

# Environment setup
RUN mkdir /home/${USERNAME}/raptor_ws
RUN apt install -y ros-noetic-ros-canopen ros-noetic-realtime-tools ros-noetic-tf libyaml-cpp-dev
