# Use the official image as a parent image
FROM wisniax/ros-rover-base

# Default configuration
ENV ROS_ENABLE_AUTOSTART=0
ENV ROS_BUILD_ON_STARTUP=on-failure
ENV ROS_ENABLE_CAN_BRIDGE=0

# Pass local directory to the container
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/raptor_ws
WORKDIR /home/${USERNAME}/raptor_ws

# Copy the local files to the container
COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}/raptor_ws
RUN rm -rf /home/${USERNAME}/raptor_ws/build
RUN rm -rf /home/${USERNAME}/raptor_ws/log
RUN rm -rf /home/${USERNAME}/raptor_ws/install

# Fix github 'unsafe' directory
RUN git config --global --add safe.directory /home/rex/raptor_ws
RUN git config --global --add safe.directory /home/rex/raptor_ws/src/rex_interfaces
RUN git config --global --add safe.directory /home/rex/raptor_ws/src/libVescCan

# Look for submodules
RUN git submodule update --init --recursive

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_start_end+ console_direct+ --executor parallel"

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
