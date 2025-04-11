# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set environment variables for ROS
ENV ROS_DISTRO=noetic
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y \
    curl wget git cmake libgl1-mesa-glx mesa-utils python3 python3-pip python3-vcstool && \
    rm -rf /var/lib/apt/lists/*   

# Update and install required dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-ros-control \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install catkin_pkg

# Set up the catkin workspace
WORKDIR /root/fira
RUN mkdir -p src
WORKDIR /root/fira/src

# Copy the project files into the container
COPY . /root/fira/src/FIRA-Air-Simulator

# Build the catkin workspace
WORKDIR /root/fira
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace in the container
RUN echo "source /root/fira/devel/setup.bash" >> ~/.bashrc

# Expose any necessary ports (adjust if needed)
EXPOSE 11311

# Set the default command to launch the simulator
CMD ["/bin/bash", "-c", "source /root/fira/devel/setup.bash && roslaunch fira_challenge_env main.launch"]