ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full
# ROS 2 Humble (Support: 23.05.2023 - 2027)

ARG USE_RVIZ=1
ARG BUILD_SEQUENTIAL=0
ENV DEBIAN_FRONTEND=noninteractive

# Workspace folder
ENV WS=/ros2_ws

# Create a user `ros` in the container
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics

SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && apt-get install -y --no-install-recommends git python3-pip software-properties-common tree wget curl \
    libusb-1.0-0-dev python3-colcon-common-extensions libpcl-dev pcl-tools libvtk9-dev libopencv-dev \
    ros-${ROS_DISTRO}-ament-cmake ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-gazebo-plugins ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-turtle-tf2-py ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-* ros-${ROS_DISTRO}-usb-cam ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-perception-pcl ros-${ROS_DISTRO}-pcl-* ros-${ROS_DISTRO}-pcl-conversions \
    libfreenect-dev ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-depth-image-proc udev openjdk-11-jre \
    && rm -rf /var/lib/apt-get/lists/*


# Download and run the install_dependencies.sh script 
RUN wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh && \ 
    chmod +x install_dependencies.sh && \ 
    ./install_dependencies.sh

ENV DEBIAN_FRONTEND=dialog

# Clone DepthAI ROS repository and install
RUN mkdir -p $WS/src
WORKDIR $WS/src
RUN git clone https://github.com/luxonis/depthai-ros.git /ros2_ws/src/depthai-ros

WORKDIR $WS
RUN rosdep update && rosdep install --from-paths src --ignore-src -y

RUN cd $WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/depthai-ros/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi

# Install pip dependencies
RUN pip install \
    depthai \
    opencv-python \
    opencv-contrib-python \
    pygame \
    numpy==1.24.4 \
    open3d \
    pandas \
    pyqtgraph \
    PyQt5 \
    ipython \
    jupyter notebook \
    ipykernel \
    nbconvert \
    setuptools==58.2.0

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config  \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video ${USERNAME}
RUN usermod -aG sudo ${USERNAME}

COPY entrypoint.sh /entrypoint.sh
COPY .bashrc /home/${USERNAME}/.bashrc
RUN chown $USERNAME:$USERNAME /home/${USERNAME}/.bashrc

# Switch from root user to `user`
USER $USERNAME

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
RUN echo "All Done!"