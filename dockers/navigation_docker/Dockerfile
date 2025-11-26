FROM ghcr.io/ros-navigation/navigation2:humble_main-1.4.0

ENV DEBIAN_FRONTEND=noninteractive

# Build argument for packages to copy (set via docker-compose)
ARG PACKAGES

WORKDIR /root/humble_ws

# Install apt dependencies and clone repos in one layer (rarely changes)
COPY dockers/navigation_docker/apt-requirements.txt dockers/navigation_docker/dependency.repos /tmp/
RUN apt-get update && \
    xargs apt-get install -y < /tmp/apt-requirements.txt && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p src && \
    vcs import src < /tmp/dependency.repos

# Copy only specific packages from humble_ws using a script
COPY humble_ws/src /tmp/humble_ws_src
RUN for pkg in $PACKAGES; do \
      if [ -d "/tmp/humble_ws_src/$pkg" ]; then \
        cp -r "/tmp/humble_ws_src/$pkg" /root/humble_ws/src/; \
      else \
        echo "Warning: Package $pkg not found in humble_ws/src"; \
      fi \
    done && \
    rm -rf /tmp/humble_ws_src

# Install ROS dependencies
COPY dockers/navigation_docker/ros-requirements.txt /tmp/ros-requirements.txt
RUN apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys "play_motion2_cli pal_sea_arm_controller_configuration pal_sea_arm_description" && \
    xargs apt-get install -y < /tmp/ros-requirements.txt && \
    rm -rf /var/lib/apt/lists/*

# # Build workspace
RUN rm -rf build install log && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source workspace in bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/humble_ws/install/setup.bash' >> /root/.bashrc

# Copy and setup entrypoint
COPY dockers/navigation_docker/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
