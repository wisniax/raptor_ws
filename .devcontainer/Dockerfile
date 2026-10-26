FROM wisniax/ros-rover-base:latest@sha256:165e355cb43b4f90348b2af6dd94dbb1b6f45f661b3edceed92221ad3c99e13b

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y libmosquitto-dev ros-jazzy-moveit-msgs

# Dockerfile for ROS-JAZZY
ARG USERNAME='rex'
ARG PASSWORD='changeme'
ARG SSH_PORT=22

# Default configuration
ENV ROS_ENABLE_AUTOSTART=0
ENV ROS_BUILD_ON_STARTUP=on-failure

# Create user
RUN useradd -m -s /bin/bash -g 1000 ${USERNAME}
RUN echo "${USERNAME}:${PASSWORD}" | chpasswd
RUN usermod -aG sudo ${USERNAME}

# Change ssh port and expose it
RUN sed -i "/^#\?Port/ { s/^#//; s/Port .*/Port ${SSH_PORT}/; }" /etc/ssh/sshd_config

# Colorful prompts ah yee
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/${USERNAME}/.bashrc

# Add important stuff to bashrc
RUN echo "\nsource /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=0\n" >> /home/${USERNAME}/.bashrc
RUN echo "source ~/raptor_ws/install/setup.bash\n" >> /home/${USERNAME}/.bashrc

# Pass local directory to the container
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/raptor_ws
WORKDIR /home/${USERNAME}/raptor_ws

# Copy the local files to the container
COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}/raptor_ws
RUN rm -rf /home/${USERNAME}/raptor_ws/build
RUN rm -rf /home/${USERNAME}/raptor_ws/log
RUN rm -rf /home/${USERNAME}/raptor_ws/install

# Fix git 'unsafe' directory
RUN git config --global --add safe.directory /home/$USERNAME/raptor_ws

# For submodules too
RUN submodules="src/rex_interfaces src/libVescCan"; \
    for subpath in $submodules; do \
        path="/home/${USERNAME}/raptor_ws/${subpath}"; \
        git config --global --add safe.directory "$path"; \
        # Only update submodule if not initialized yet
        if [ ! -d "$path" ] || [ -z "$(find "$path" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then \
            git submodule update --init -- "$path"; \
        fi; \
    done

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_start_end+ console_direct+ --executor parallel"

# Prepare .ssh directory for public keys
RUN rm -rf /home/${USERNAME}/.ssh
RUN mkdir -m 0700 /home/${USERNAME}/.ssh

USER root
COPY .devcontainer/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

CMD [ "/usr/local/bin/docker-entrypoint.sh" ]
