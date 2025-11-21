FROM ghcr.io/ros-navigation/navigation2:humble_main-1.4.0

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root/nav2_ws

# Install apt dependencies and clone repos in one layer (rarely changes)
COPY docker/apt-requirements.txt docker/dependency.repos /tmp/
RUN apt-get update && \
    xargs apt-get install -y < /tmp/apt-requirements.txt && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p src && \
    vcs import src < /tmp/dependency.repos

# Copy workspace 
COPY nav2_ws /root/nav2_ws

# Install ROS dependencies
COPY docker/ros-requirements.txt /tmp/ros-requirements.txt
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
    echo 'source /root/nav2_ws/install/setup.bash' >> /root/.bashrc

# Copy and setup entrypoint
COPY docker/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/root/entrypoint.sh"]
