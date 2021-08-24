ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.4.3
FROM ${BASE_IMAGE}
# Size: 631MB

LABEL maintainer="9426224" \
      version="0.1" \
      description="A l4t-base dockerfile for ROS2 User" \
      email="9426224@live.com"

ENV DEBIAN_FRONTEND=noninteractive

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /tmp

# 
# Description: Change the locale from POSIX to UTF-8
# Size: 3.42MB
#
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8


# 
# Description: Install basic packages and Add ROS repo
# Size: 239MB
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
        build-essential \
		git \
        openssh-server \
        vim \
        usbutils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /tmp/*


# 
# Description: Build CMake 3.20.5
# Size: 120MB
#
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate  \
        https://github.com/Kitware/CMake/releases/download/v3.20.5/cmake-3.20.5-linux-aarch64.tar.gz &&\
    mv ./cmake-3.20.5-linux-aarch64.tar.gz ./cmake-3.20.tar.gz && \
    tar -zxvf ./cmake-3.20.tar.gz -C /usr/local/share/ && \
    ln -s /usr/local/share/cmake-3.20 /usr/bin/cmake && \
    ln -s /usr/local/share/cmake-3.20 /usr/local/bin/cmake && \
    rm -rf /tmp/*


# 
# Description: Install Development packages
# Size: 438MB
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libtool \
        apt-utils \
        autoconf \
        automake \
        unzip \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
        libopenblas-base \
        libopenmpi-dev \
        libopenblas-dev \
        libatlas-base-dev \
        liblapack-dev \
        libboost-python-dev \
        libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && rm -rf /tmp/*


# 
# Description: Install Python Test Environment
# Size: 17.9MB
#
RUN python3 -m pip install --no-cache-dir -U \
        wheel && \
    python3 -m pip install --no-cache-dir -U \
        Cython \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest && \
    rm -rf /tmp/* && \
    rm -rf ~/.cache/pip


# 
# Description: Link CUDART to ENV
# Size: 118kB
#
RUN ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/libcudart.so.10.2 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/libnvrtc.so /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/libnvrtc.so.10.2 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/libnvrtc-builtins.so /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/local/cuda-10.2/targets/aarch64-linux/lib/libcudart.so.10.2.89 /usr/lib/aarch64-linux-gnu/ && \
    ln -s $(which nvcc) /sbin/nvcc


# 
# Description: Install yaml
# Size: 5.01MB
#
RUN git clone --branch yaml-cpp-0.7.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.7 && \
    cd yaml-cpp-0.7 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    make -j$(nproc) install && \
    rm -rf /tmp/*


# 
# Description: Install OpenCV 4.5.0 with CUDA
# Size: 466MB
#
RUN apt-get purge -y '*opencv*' || echo "previous OpenCV installation not found" && \
    mkdir opencv && \
    cd opencv && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz -O OpenCV-4.5.0-aarch64.tar.gz && \
    tar -xzvf OpenCV-4.5.0-aarch64.tar.gz && \
    dpkg -i --force-depends *.deb && \
    apt-get update && \
    apt-get install -y -f --no-install-recommends && \
    dpkg -i *.deb && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    rm -rf /tmp/* && \
    cp -r /usr/include/opencv4 /usr/local/include/opencv4 && \
    cp -r /usr/lib/python3.6/dist-packages/cv2 /usr/local/lib/python3.6/dist-packages/cv2


# 
# Description: Install LibTorch 1.7.0
# Size: 620MB
#
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \
        https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl && \
    pip3 install --no-cache-dir torch-1.7.0-cp36-cp36m-linux_aarch64.whl && \
    rm -rf ~/.cache/pip && \
    rm -rf /tmp/*


# 
# Description: Install Protobuf 3.8.0
# Size: 307MB
#
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \
        https://github.com/protocolbuffers/protobuf/releases/download/v3.8.0/protobuf-python-3.8.0.zip && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \
        https://github.com/protocolbuffers/protobuf/releases/download/v3.8.0/protoc-3.8.0-linux-aarch_64.zip && \
    unzip protobuf-python-3.8.0.zip && \
    unzip protoc-3.8.0-linux-aarch_64.zip -d protoc-3.8.0 && \
    cp protoc-3.8.0/bin/protoc /usr/local/bin/protoc && \
    export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp && \
    cd protobuf-3.8.0/ && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local && \
    make -j $(nproc) && \
    make check && \
    make -j $(nproc) install && \
    ldconfig && \
    cd python/ && \
    python3 setup.py build --cpp_implementation && \
    python3 setup.py test --cpp_implementation && \
    sudo python3 setup.py install --cpp_implementation && \
    rm -rf /tmp/*


# 
# Description: Install PyCuda 2019.1.2 ONNX 1.4.1
# Size: 26.4MB
#
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \
        https://files.pythonhosted.org/packages/5e/3f/5658c38579b41866ba21ee1b5020b8225cec86fe717e4b1c5c972de0a33c/pycuda-2019.1.2.tar.gz && \
    tar xzvf pycuda-2019.1.2.tar.gz && \
    cd pycuda-2019.1.2 && \
    python3 ./configure.py --python-exe=/usr/bin/python3 --cuda-root=/usr/local/cuda --cudadrv-lib-dir=/usr/lib/aarch64-linux-gnu --boost-inc-dir=/usr/include --boost-lib-dir=/usr/lib/aarch64-linux-gnu --boost-python-libname=boost_python3-py36 --boost-thread-libname=boost_thread --no-use-shipped-boost && \
    make -j$(nproc) && \
    python3 setup.py build && \
    python3 setup.py install && \
    pip3 install --no-cache-dir onnx==1.4.1 && \
    rm -rf ~/.cache/pip && \
    rm -rf /tmp/*


# # 
# # Description: Install ROS2 Foxy (Build from Ubuntu 18.04 Source)
# # Size: 729MB
# #
RUN mkdir -p /opt/ros/foxy/src && \
    cd /opt/ros/foxy && \
    rosinstall_generator --deps --rosdistro foxy ros_base \
        launch_xml \
        launch_yaml \
        launch_testing \
        launch_testing_ament_cmake \
        demo_nodes_cpp \
        demo_nodes_py \
        example_interfaces \
        camera_calibration_parsers \
        camera_info_manager \
        cv_bridge \
        v4l2_camera \
        vision_opencv \
        vision_msgs \
        image_geometry \
        image_pipeline \
        image_transport \
        compressed_image_transport \
        compressed_depth_image_transport \
    > ros2.foxy.ros_base.rosinstall && \
    cat ros2.foxy.ros_base.rosinstall && \
    vcs import src < ros2.foxy.ros_base.rosinstall && \
    rm ./src/libyaml_vendor/CMakeLists.txt && \
    wget --no-check-certificate https://raw.githubusercontent.com/ros2/libyaml_vendor/master/CMakeLists.txt -P ./src/libyaml_vendor/ && \
    apt-get update && \
    cd /opt/ros/foxy && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
        --ignore-src \
        --from-paths src \
        --rosdistro foxy \
        --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    colcon build --merge-install && \
    rm -rf /opt/ros/foxy/src && \
    rm -rf /opt/ros/foxy/log && \
    rm -rf /opt/ros/foxy/build && \
    rm /opt/ros/foxy/*.rosinstall && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    rm -rf /tmp/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


# 
# Description: Add Node.js
# Size: 326MB
#
RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \ 
        https://nodejs.org/dist/v14.17.5/node-v14.17.5-linux-arm64.tar.xz && \
    tar -xvf node-v14.17.5-linux-arm64.tar.xz && \
    mv node-v14.17.5-linux-arm64 /opt/node && \
    ln -s /opt/node/bin/node /usr/local/bin/node && \
    ln -s /opt/node/bin/npm /usr/local/bin/npm && \
    npm install -g pm2 && \
    ln -s /opt/node/bin/pm2 /usr/local/bin/ && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \  
        http://downloads.mongodb.org/linux/mongodb-linux-aarch64-ubuntu1804-4.4.5.tgz && \
    tar -zxvf mongodb-linux-aarch64-ubuntu1804-4.4.5.tgz && \
    mv mongodb-linux-aarch64-ubuntu1804-4.4.5 /usr/local/mongodb && \
    echo "export PATH=/usr/local/mongodb/bin:\$PATH" >> /etc/profile && \
    source /etc/profile && \
    mkdir -p /usr/local/mongodb/data/db && \
    touch /usr/local/mongodb/data/log && \
    chmod 777 -R /usr/local/mongodb/data && \
    ln -s /usr/local/mongodb/bin/mongod /usr/bin/mongod && \
    ln -s /usr/local/mongodb/bin/mongo /usr/bin/mongo && \
    touch /usr/local/mongodb/bin/mongodb.conf && \
    echo $'dbpath = /usr/local/mongodb/data/db\n \
    logpath = /usr/local/mongodb/data/log\n \
    logappend = true\n \
    port = 27017\n \
    fork = true\n \
    bind_ip = 0.0.0.0\n \
    quiet = true\n \
    auth = false' > /usr/local/mongodb/bin/mongodb.conf && \
    touch /usr/local/mongodb/db-install.js && \
    echo $'var url = "mongodb://localhost:27017/admin"\n \
    var db = connect(url)\n \
    db.createUser({user:"bitcq",pwd:"bitcqpwd",roles: [ { role: "root", db: "admin" } ]})\n \
    var url1 = "mongodb://localhost:27017/ecu_db"\n \
    var db1 = connect(url1)\n \
    db1.users.insert({"username":"admin","password":"e10adc3949ba59abbe56e057f20f883e","authority":1});\n \
    db1.users.ensureIndex({ username: 1 }, { unique: true })\n \
    var url2 = "mongodb://localhost:27017/mec_db"\n \
    var db2 = connect(url2)\n \
    db2.users.insert({ "username": "admin", "password": "e10adc3949ba59abbe56e057f20f883e", "authority": 1 });\n \
    db2.users.ensureIndex({ username: 1 }, { unique: true })' > /usr/local/mongodb/db-install.js && \
    rm -rf /tmp/*


# 
# Description: Add EntryPoint and .bashrc\SSH params.
# Size: 148KB
#
RUN echo $'#!/bin/bash\n \
    set -e\n \
    /etc/init.d/ssh start\n \
    mongod --config /usr/local/mongodb/bin/mongodb.conf\n \
    exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh && \
    echo 'source /opt/ros/foxy/install/setup.bash' >> /root/.bashrc && \
    echo $'PermitRootLogin yes\nPubkeyAuthentication yes' >> /etc/ssh/sshd_config


EXPOSE 22
EXPOSE 8001
EXPOSE 63029


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /root/

# sudo docker volume create mongodbdata
# sudo docker run -itd -p 6622:22 -p 8001:8001 -p 63029:63029 --privileged -v mongodbdata:/usr/local/mongodb/data --name="ros-foxy" --restart=always 9426224/ros-foxy-base:latest
# sudo docker exec -it ros-foxy /bin/bash