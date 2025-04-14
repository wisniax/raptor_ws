# Use the official image as a parent image
FROM ros:jazzy-perception

# Default configuration
ENV ROS_ENABLE_AUTOSTART=1
ENV ROS_BUILD_ON_STARTUP=1

# Dockerfile for ROS-JAZZY
ARG USERNAME='rex'
ARG PASSWORD='changeme'

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano can-utils ssh libpaho-mqttpp-dev libpaho-mqtt-dev less iputils-ping

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

# Add important stuff to bashrc
RUN echo "\nsource /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=1\n" >> /home/${USERNAME}/.bashrc
RUN echo "source ~/raptor_ws/install/setup.bash\n" >> /home/${USERNAME}/.bashrc

RUN apt update && apt upgrade -y

# Environment setup
RUN mkdir /home/${USERNAME}/raptor_ws
RUN apt install -y ros-jazzy-ros2-socketcan libasio-dev libceres-dev ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-updater ros-jazzy-rtcm-msgs ros-jazzy-nmea-msgs

RUN usermod -a -G dialout ${USERNAME}

USER ${USERNAME}
RUN git config --global --add safe.directory /home/rex/raptor_ws
USER root

# Add Tini - its cool I think
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini
ENTRYPOINT ["/tini", "--"]

COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# CMD ["/bin/bash", "-c", "rm -f /tmp/rexlaunch.pgid && service ssh start && exec service rex docker-entrypoint"]
CMD [ "/usr/local/bin/docker-entrypoint.sh" ]