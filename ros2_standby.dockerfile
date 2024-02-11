ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}-ros-core
# docker pull ros:galactic-ros-core
# -ros-base
# shaderobotics/yolov5:${ROS_DISTRO}
# The ARG using before FROM only works on FROM. Define a new ENV or ARG after FROM.
# ARG 指令有生效范围，如果在 FROM 指令之前指定，那么只能用于 FROM 指令中。
ENV ROS_DISTRO galactic

WORKDIR /app

# COPY rosbag/rosbag2_20_09_08 /app/rosbag/rosbag2_20_09_08

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-topic-tools &&\
    rm -rf /var/lib/apt/lists/* 


# CMD ["ros2", "bag", "record", "-o", "k3d_result" "-a"]
