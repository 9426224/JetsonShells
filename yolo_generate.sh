#!/bin/bash
sudo service polipo restart 

has_src=$(rm -rf src)
has_docker=$(sudo docker rm -f galactic)

mkdir src
cd src
git clone --branch ros-galactic https://sw9426224:f86815ba0c0befb27143581b592c1955@gitee.com/bitcq/bitcq_msg.git
git clone --branch ros-galactic https://sw9426224:f86815ba0c0befb27143581b592c1955@gitee.com/bitcq/yolov4_trt.git
cd ..

sudo docker run -itd \
    --runtime nvidia \
    --restart=always \
    --net=host \
    --ipc=host \
    --privileged \
    -e cspwd=bitcq \
    -e csport=60001 \
    -e sshport=60000 \
    -e rootpwd=root \
    --name="galactic" \
    9426224/ros-galactic-develop

sleep 5

sudo docker cp src galactic:/root/ros2_ws/

sudo docker exec -it galactic /root/ros2_ws/src/yolov4_trt/shell/generate_release.sh $1

sudo docker cp galactic:/root/ros2_ws/yolov4-trt_$1_arm64.deb ./docker-release-pack/yolo_release/

sudo docker rm -f galactic
