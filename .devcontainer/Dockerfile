# Use the official image as a parent image
FROM ros:jazzy-perception

# Default configuration
ENV ROS_ENABLE_AUTOSTART=0
ENV ROS_BUILD_ON_STARTUP=on-failure
ENV ROS_ENABLE_CAN_BRIDGE=0

# Dockerfile for ROS-JAZZY
ARG USERNAME='rex'
ARG PASSWORD='changeme'

# Update the system
RUN apt update && apt upgrade -y

# Install random stuff
RUN DEBIAN_FRONTEND=noninteractive apt install -y git gdb nano curl wget python3 python3-pip net-tools apt-utils nano can-utils ssh libpaho-mqttpp-dev libpaho-mqtt-dev less iputils-ping bc rapidjson-dev iproute2

# Random Stuff
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN ln -sf /usr/share/zoneinfo/Europe/Warsaw /etc/localtime

# Create user
RUN useradd -m -s /bin/bash -g 1000 ${USERNAME}
RUN echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN usermod -aG sudo ${USERNAME}

# Disable sudo password auth
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Colorful prompts ah yee
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

# Add important stuff to bashrc
RUN echo "\nsource /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=1\n" >> /home/${USERNAME}/.bashrc
RUN echo "source ~/raptor_ws/install/setup.bash\n" >> /home/${USERNAME}/.bashrc

RUN apt update && apt upgrade -y

# Environment setup
RUN apt install -y ros-jazzy-ros2-socketcan libasio-dev libceres-dev ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-updater ros-jazzy-rtcm-msgs ros-jazzy-nmea-msgs

# Pass local directory to the container
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/raptor_ws
WORKDIR /home/${USERNAME}/raptor_ws

# Copy the local files to the container
COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}/raptor_ws
RUN rm -rf /home/${USERNAME}/raptor_ws/build
RUN rm -rf /home/${USERNAME}/raptor_ws/log
RUN rm -rf /home/${USERNAME}/raptor_ws/install
RUN ls -al

# Fix github 'unsafe' directory
RUN git config --global --add safe.directory /home/rex/raptor_ws
RUN git config --global --add safe.directory /home/rex/raptor_ws/src/rex_interfaces
RUN git config --global --add safe.directory /home/rex/raptor_ws/src/libVescCan

# Look for submodules
RUN git submodule update --init --recursive

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Manage public ssh keys
RUN rm -rf /home/${USERNAME}/.ssh
RUN mkdir -m 0700 /home/${USERNAME}/.ssh
RUN touch /home/${USERNAME}/.ssh/authorized_keys
RUN chmod 640 /home/${USERNAME}/.ssh/authorized_keys
ADD --chown=${USERNAME}:${USERNAME} ./.devcontainer/public_keys.d /home/${USERNAME}/.ssh/public_keys.d
RUN cat /home/${USERNAME}/.ssh/public_keys.d/*.pub >> /home/${USERNAME}/.ssh/authorized_keys || true
RUN rm -rf /home/${USERNAME}/.ssh/public_keys.d

USER root
COPY .devcontainer/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

CMD [ "/usr/local/bin/docker-entrypoint.sh" ]