#!/bin/bash
sudo service polipo restart 

has_src=$(rm -rf src)
has_docker=$(sudo docker rm -f galactic)

mkdir src
cd src
git clone --branch intra https://sw9426224:f86815ba0c0befb27143581b592c1955@gitee.com/bitcq/bitcq_msg.git
git clone --branch intra https://sw9426224:f86815ba0c0befb27143581b592c1955@gitee.com/bitcq/bitcq_ca.git bitcq_camera
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

sudo docker exec galactic /root/ros2_ws/src/bitcq_camera/shell/generate_release.sh $1

sudo docker cp galactic:/root/ros2_ws/bitcq-camera_$1_arm64.deb ./docker-release-pack/bitcq_camera_release/

sudo docker rm -f galactic
