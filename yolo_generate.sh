#!/bin/bash -e

sudo docker exec -it $1 /root/ros2_ws/src/yolov4_trt/shell/generate_release.sh $2

sudo docker cp $1:/root/ros2_ws/yolov4-trt_$2_arm64.deb ./docker-release-pack/yolo_release/
