# Use the official ROS Noetic base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-wstool \
    python3-catkin-tools \
    git \
    libboost-all-dev \
    libboost-system-dev \
    ros-noetic-costmap-2d \
    ros-noetic-rviz \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-rviz-plugin-tutorials \
    gstreamer1.0-plugins-good \
    python-is-python3 \
    ros-noetic-axis-camera \
    ros-noetic-spacenav-node \
    ros-noetic-teleop-tools \
    ros-noetic-controller-manager \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-rospkg \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-navigation \
    ros-noetic-slam-toolbox \
    ros-noetic-twist-mux \
    ros-noetic-laser-filters \
    libboost-all-dev \
    ros-noetic-xacro \
    ros-noetic-roslint  

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Create and set the working directory
WORKDIR /root/ros_ws

# RUN mkdir -p src


# ENV BOOST_INCLUDEDIR=/usr/include
# ENV BOOST_LIBRARYDIR=/usr/lib/aarch64-linux-gnu
RUN git config --global http.postBuffer 524288000
RUN wstool init src
COPY ./.rosinstall ./src/.rosinstall
RUN wstool update -t src
COPY ./src ./src
# RUN rosdep install --from-paths src -i --rosdistro noetic -y

RUN chmod -R +x /root/ros_ws/src


RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  cd src && \
                  catkin_init_workspace && \
                  cd .. && \
                  catkin_make"


RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

RUN /bin/bash -c "source ./devel/setup.bash"

CMD ["/bin/bash"]