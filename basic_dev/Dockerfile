FROM ghcr.io/irobot-algorithm/sentry_navigation/environment:latest
ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

RUN apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
 ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
 ros-noetic-roslint net-tools ros-noetic-mavros liborocos-bfl-dev

ENV ROS_DISTRO noetic

WORKDIR /basic_dev/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/livox_ros_driver2/build.sh ROS1 && \ 
    catkin_make -DCATKIN_WHITELIST_PACKAGES="airsim_ros" && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="" && \ 
    catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# ENTRYPOINT [ "/setup.bash" ]