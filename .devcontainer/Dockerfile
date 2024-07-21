# Use the official image as a parent image
FROM osrf/ros:noetic-desktop-full

# Dockerfile for ROS-NOETIC

ARG USERNAME='vscode'
ARG PASSWORD='vscode'

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano
# gedit vim neovim firefox

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

# Add alias vnc to start the vnc lmao
RUN echo "\n alias vnc='sudo /usr/local/share/desktop-init.sh' \n source /opt/ros/noetic/setup.sh" >> /home/${USERNAME}/.bashrc

# Environment setup
RUN mkdir /home/${USERNAME}/raptor_ws
RUN apt install -y ros-noetic-ros-canopen

# Install Google Chrome
RUN curl -fSsL https://dl.google.com/linux/linux_signing_key.pub | gpg --dearmor | sudo tee /usr/share/keyrings/google-chrome.gpg > /dev/null
RUN echo deb [arch=amd64 signed-by=/usr/share/keyrings/google-chrome.gpg] http://dl.google.com/linux/chrome/deb/ stable main | sudo tee /etc/apt/sources.list.d/google-chrome.list
RUN apt update && apt install -y google-chrome-stable