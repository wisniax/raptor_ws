# Use the official image as a parent image
FROM osrf/ros:jazzy-desktop-full

# Dockerfile for ROS-JAZZY

ARG USERNAME='rex'
ARG PASSWORD='changeme'

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano can-utils ssh libpaho-mqttpp-dev libpaho-mqtt-dev

# Random Stuff
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN ln -sf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime

# Create user
RUN useradd -m -s /bin/bash -g 1000 ${USERNAME}
RUN echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN usermod -aG sudo ${USERNAME}

# Colorful prompts ah yee
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

# # paho-mqtt libraries (for mqtt_bridge)
# WORKDIR /tmp
# RUN git clone https://github.com/eclipse/paho.mqtt.cpp
# WORKDIR /tmp/paho.mqtt.cpp
# RUN git checkout v1.4.0
# RUN git submodule init
# RUN git submodule update
# RUN cmake -Bbuild -H. -DPAHO_WITH_MQTT_C=ON -DPAHO_BUILD_EXAMPLES=ON
# RUN sudo cmake --build build/ --target install

# Add important stuff to bashrc
# RUN echo "\nexport ROS_MASTER_URI=http://127.0.0.1:11311\nexport ROS_HOSTNAME=127.0.0.1\n" >> /home/${USERNAME}/.bashrc
# RUN echo "\nsource /opt/ros/jazzy/setup.bash\nsource /home/rex/raptor_ws/devel/setup.bash\n" >> /home/${USERNAME}/.bashrc
RUN echo "\nsource /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=1\n" >> /home/${USERNAME}/.bashrc
RUN echo "source ~/raptor_ws/install/setup.bash\n" >> /home/${USERNAME}/.bashrc

# Add alias vnc to start the vnc lmao
RUN echo "\nalias vnc='sudo /usr/local/share/desktop-init.sh'" >> /home/${USERNAME}/.bashrc

# Environment setup
RUN mkdir /home/${USERNAME}/raptor_ws
RUN apt install -y ros-jazzy-ros2-socketcan ros-jazzy-realtime-tools libceres-dev
# RUN apt install -y ros-noetic-ros-canopen ros-noetic-realtime-tools ros-noetic-tf libyaml-cpp-dev
