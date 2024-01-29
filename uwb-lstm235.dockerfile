ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}-ros-core
# docker pull ros:galactic-ros-core
# -ros-base
# shaderobotics/yolov5:${ROS_DISTRO}
# The ARG using before FROM only works on FROM. Define a new ENV or ARG after FROM.
# ARG 指令有生效范围，如果在 FROM 指令之前指定，那么只能用于 FROM 指令中。
ENV ROS_DISTRO galactic

WORKDIR /app

COPY requirements_lstm.txt /app
COPY uwb-lstm-docker235.py /app
COPY utlis /app/utlis

RUN mkdir -p /app/models && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        git \
        python-dateutil && \
    rm -rf /var/lib/apt/lists/*  && \
    python3 -m pip install --no-cache-dir -r requirements_lstm.txt && \
    python3 -m pip install python-dateutil 

# WORkDIR /app/results/results_csv/triangulation/computation
# WORkDIR /app/results/results_csv/triangulation/pos/pos_tri
# WORkDIR /app/models
# RUN mkdir -p /app/results/results_csv/triangulation/computation \
#              /app/results/results_csv/triangulation/pos/pos_tri \
#              /app/models
# COPY lstm_uwb_0 /app/models/lstm_uwb_0
COPY models/lstm_uwb_2_5 /app/models/lstm_uwb_2_5
COPY models/lstm_uwb_3_5 /app/models/lstm_uwb_3_5

CMD ["python3", "/app/uwb-lstm-docker235.py"]

####### , "--with_model", "False", "--poses_save", "--computation_save"


# use a shell script to do source and export
# COPY entrypoint_ros2_dockerfile.sh /app/yolo_ws/entrypoint_ros2_dockerfile.sh

# RUN apt-get clean && apt-get autoclean && apt-get autoremove

# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source ./install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && ros2 run yolov5_ros2 yolo_detect --ros-args -p image_topic:=/${IMAGE_TOPIC} -p model:=${YOLO_MODEL}"]
# ENTRYPOINT ["/app/yolo_ws/entrypoint_ros2_dockerfile.sh"]
# CMD ["--ROS_DISTRO=galactic", "--ROS_DOMAIN_ID=7", "--IMAGE_TOPIC=image_raw", "--YOLO_MODEL=yolov5s"]
# CMD ["-R", "galactic", "-I", "7", "-T", "image_raw", "-M", "yolov5s", "-D", "cpu"]