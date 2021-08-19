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
RUN apt update && \
    apt install -y --no-install-recommends \
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
# RUN wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate \
#     https://cmake.org/files/v3.20/cmake-3.20.3.tar.gz && \
#     tar -xzvf cmake-3.20.3.tar.gz && \
#     cd cmake-3.20.3 && \
#     ./bootstrap && \
#     make -j ${nproc} && \
#     make -j ${nproc} install && \
#     rm /usr/bin/cmake && \
#     ln -s /usr/local/bin/cmake /usr/bin/cmake && \
#     rm -rf /tmp/*


# 
# Description: Install Development packages
# Size: 245MB
#
RUN apt update && \
    apt install -y --no-install-recommends \
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
#		libgazebo9-dev \
#		gazebo9 \
#		gazebo9-common \
#		gazebo9-plugin-base \
        libopenblas-base \
        libopenmpi-dev \
        libopenblas-dev \
        libatlas-base-dev \
        liblapack-dev \
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


CMD ["bash"]
WORKDIR /
