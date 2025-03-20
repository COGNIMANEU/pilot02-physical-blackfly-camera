# Use the official ROS 2 Humble image as a base
FROM ros:humble

# Install essential dependencies for building ROS 2 packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    wget \    
    ros-humble-std-msgs \
    ros-humble-camera-info-manager \ 
    libpcl-dev \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    ros-humble-rclcpp \
    ros-humble-cv-bridge \
    ros-humble-ament-cmake-clang-format \  
    && rm -rf /var/lib/apt/lists/*


# Create a workspace and copy the source code into the container
WORKDIR /workspace
COPY ./src /workspace/src

# Build the workspace using colcon
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the workspace setup script by default
RUN echo 'source /workspace/install/setup.bash' >> /root/.bashrc

# Set environment variables 
ENV LAUNCH_FILE=master_example.launch.py  
ENV CAMERAMODULE=spinnaker_synchronized_camera_driver

# Copy config and launch folders from the host to the container (ensure these paths exist on the host)
COPY ./config /workspace/src/
COPY ./launch /workspace/src/

# Default command to source the workspace and run a specified launch file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch $CAMERAMODULE $LAUNCH_FILE"]