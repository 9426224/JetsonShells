#!/bin/bash

##############################################
# FunctionName:echocolor
##############################################
function echocolor() {
    string=$1
    color=$2
    currentTimestamp=$(date "+%Y-%m-%d %H:%M:%S")

    ## String
    if [ "x${string}" == "x" ]; then
        string=""
    else
        string="${currentTimestamp} ${string}"
    fi

    ## Color
    #Red : Error Message
    if [ "${color}" == "red" ]; then
        string="\n\033[1;41;37m ${string} \033[0m"

    #Green : Success Message
    elif [ "${color}" == "green" ]; then
        string="\033[1;42;37m ${string} \033[0m"

    #Yellow : Warning Message
    elif [ "${color}" == "yellow" ]; then
        string="\033[1;43;37m ${string} \033[0m"

    #Blue : Question Message
    elif [ "${color}" == "blue" ]; then
        string="\033[1;44;37m ${string} \033[0m"

    #Purple : Module Install Start
    elif [ "${color}" == "purple" ]; then
        string="\033[1;45;37m ${string} \033[0m"

    #Cyan : Version Message
    elif [ "${color}" == "cyan" ]; then
        string="\033[1;46;37m ${string} \033[0m"

    else
        string="\033[1;40;37m ${string} \033[0m"
    fi

    echo -e "${string}"
}

##############################################
# FunctionName:version_** 
##############################################
function version_gt() { test "$(echo "$@" | tr " " "\n" | sort -V | head -n 1)" != "$1"; } #>
function version_le() { test "$(echo "$@" | tr " " "\n" | sort -V | head -n 1)" == "$1"; } #<=
function version_lt() { test "$(echo "$@" | tr " " "\n" | sort -rV | head -n 1)" != "$1"; } #<
function version_ge() { test "$(echo "$@" | tr " " "\n" | sort -rV | head -n 1)" == "$1"; } #>=

##############################################
# FunctionName:UpdateSource
##############################################
function UpdateSource() {
    echocolor "Update Source" "purple"
    
    mv /etc/apt/sources.list{,.bak}
    rm /etc/apt/source.list.bak

    [ -f /etc/apt/sources.list ] && rm /etc/apt/sources.list

    echocolor "Choose which source you want use: (A/T/U/S/N) (A:Aliyun T:TsingHua U: Ustc S:Ubuntu Source N:Do not update source)" "blue"
    read sourceChoose
    if [[ "$sourceChoose" = "N" ]] || [[ "$sourceChoose" = "n" ]]; then
        cp -ax /etc/apt/sources.list.bak /etc/apt/sources.list
        echocolor "Doesn't update source" "yellow"
    elif [[ "$sourceChoose" = "A" ]] || [[ "$sourceChoose" = "a" ]]; then
        echo "deb https://mirrors.aliyun.com/ubuntu-ports/ bionic main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb https://mirrors.aliyun.com/ubuntu-ports/ bionic-security main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb https://mirrors.aliyun.com/ubuntu-ports/ bionic-updates main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb https://mirrors.aliyun.com/ubuntu-ports/ bionic-backports main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src https://mirrors.aliyun.com/ubuntu-ports/ bionic main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src https://mirrors.aliyun.com/ubuntu-ports/ bionic-security main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src https://mirrors.aliyun.com/ubuntu-ports/ bionic-updates main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src https://mirrors.aliyun.com/ubuntu-ports/ bionic-backports main multiverse restricted universe" >>/etc/apt/sources.list
        echocolor "Ubuntu source has been updated to Aliyun source." "green"
    elif [[ "$sourceChoose" = "T" ]] || [[ "$sourceChoose" = "t" ]]; then
        echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main multiverse restricted universe" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main multiverse restricted universe" >>/etc/apt/sources.list
        echocolor "Ubuntu source has been updated to Tsinghua source." "green"
    elif [[ "$sourceChoose" = "U" ]] || [[ "$sourceChoose" = "u" ]]; then
        echo "deb http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://mirrors.ustc.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse" >>/etc/apt/sources.list
        echocolor "Ubuntu source has been updated to Ustc source." "green"
    elif [[ "$sourceChoose" = "S" ]] || [[ "$sourceChoose" = "s" ]]; then
        echo "deb http://ports.ubuntu.com/ bionic main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://ports.ubuntu.com/ bionic-security main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted" >>/etc/apt/source.list
        echo "deb-src http://ports.ubuntu.com/ bionic main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://ports.ubuntu.com/ bionic-security main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse" >>/etc/apt/sources.list
        echo "deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted" >>/etc/apt/source.list
        echocolor "Ubuntu source has been updated to Original source." "green"
    fi

    echocolor "Add New ros apt-key to keyserver: (Y/N)" "blue"
    read keyChoose
    if [[ "$keyChoose" = "Y" ]] || [[ "$keyChoose" = "y" ]]; then
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        if [ $? -eq 1 ]; then
            curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
        fi
    fi

    if [[ "$sourceChoose" != "N" ]] && [[ "$sourceChoose" != "n" ]]; then
        $systemPackage update
        if [ $? -eq 1 ]; then
            echocolor "Get apt update list failed." "red"
        fi
    fi

    echo
}

##############################################
# FunctionName:InstallSimpleProgram
##############################################
function InstallSimpleProgram() {
    echo
    program=$1
    echocolor "Install $program" "purple"

    programName=$(dpkg -l | grep $program | awk 'NR==1 {print $2 " " $3}')
    if [[ -z $programName ]]; then
        $systemPackage -y install $program
        if [ $? -ne 0 ]; then
            echocolor "$program install failed, try re-install..." "red"
            $systemPackage -y install $program
            if [ $? -ne 0 ]; then
                echocolor "$program install failed." "red"
            else
                echocolor "$program has been installed." "green"
            fi
        else
            echocolor "$program has been installed." "green"
        fi
    else
        echocolor "$program already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallROS
##############################################
function InstallROS() {
    echo
    echocolor "Install ROS" "purple"

    echocolor "Do you want to install ROS? (Y/N)" "blue"
    read rosInstall
    if [ "$rosInstall" != "Y" ] && [ "$rosInstall" != "y" ];then
        echocolor "Install ROS Failed."  "red"
        return
    fi

    #ROS dependencies
    InstallSimpleProgram "python-rosdep"
    InstallSimpleProgram "python-rosinstall"
    InstallSimpleProgram "python-rosinstall-generator"
    InstallSimpleProgram "python-wstool"
    InstallSimpleProgram "python-catkin-tools"

    which roscore
    if [ $? -eq 1 ];then
        InstallSimpleProgram "ros-melodic-desktop-full"

        echo "source /opt/ros/melodic/setup.bash" >>~/.bashrc
        source ~/.bashrc
        echocolor "Add ros source to .bashrc successfully." "green"

        rosdep init
        if [ $? -eq 1 ]; then
            wget http://packages.ros.org/ros.key -O - | apt-key add -
            rosdep init
        fi
        rosdep update
        if [ $? -eq 1 ]; then
            echocolor "Rosdep init failed." "red"
        fi
    else
        echocolor "ROS already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallOpenCV
##############################################
function InstallOpenCV() {
    echo
    echocolor "Install OpenCV" "purple"

    echocolor "Do you want to install OpenCV? (Y/N)" "blue"
    read opencvInstall
    if [ "$opencvInstall" != "Y" ] && [ "$opencvInstall" != "y" ];then
        echocolor "Install OpenCV Failed."  "red"
        return
    fi

    #OpenCV dependencies
    InstallSimpleProgram "build-essential"
    InstallSimpleProgram "libgtk2.0-dev"
    InstallSimpleProgram "libavcodec-dev"
    InstallSimpleProgram "libavformat-dev"
    InstallSimpleProgram "libjpeg-dev"
    InstallSimpleProgram "libtiff-dev"
    InstallSimpleProgram "libswscale-dev"
    InstallSimpleProgram "libjasper-dev"

    #opencv_version
    opencvVersion=$(pkg-config opencv --modversion)
    version=3.4.14

    pkg-config opencv --modversion
    if [ $? -eq 1 ];then
        cd ${HOME}/src

        if [[ -f ./opencv-$version.tar.gz ]]; then
            echocolor "opencv-$version.tar.gz exists" "yellow"
        else
            wget -c https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/$version -O opencv-$version.tar.gz
            if [ $? -eq 1 ]; then
                echocolor "Download OpenCV $version source code failed." "red"
                exit
            else
                echocolor "Download Succeed." "green"
            fi
        fi

        if [[ -f ./opencv-$version.tar.gz ]]; then
            tar -zxf opencv-$version.tar.gz
            if [ $? -eq 1 ]; then
                echocolor "Unzip OpenCV failed, may file was wrong or broken, try delete and re-run scripts." "red"
                exit
            else
                echocolor "Unzip OpenCV Success." "green"
            fi
        fi

        cd ./opencv-$version && mkdir -p build && cd build
        cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_TESTS=OFF -DWITH_CUDA=ON -DWITH_CUDNN=ON -DOPENCV_DNN_CUDA=ON -DCUDA_FAST_MATH=1 -DCUDA_ARCH_BIN=7.5 -DWITH_CUBLAS=1 -DENABLE_FAST_MATH=1 -DWITH_TBB=ON -DBUILD_NEW_PYTHON_SUPPORT=ON -DWITH_V4L=ON -DBUILD_TIFF=ON -DINSTALL_C_EXAMPLES=OFF -DINSTALL_PYTHON_EXAMPLES=OFF -DBUILD_EXAMPLES=OFF -DWITH_GSTREAMER=ON -DWITH_GTK=ON -DWITH_GTHREAD=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_FFMPEG=ON -DWITH_LIBV4L=ON -DBUILD_NEW_PYTHON_SUPPORT=ON -DHAVE_opencv_python3=ON -DBUILD_EXAMPLES=OFF ..
        if [ $? -eq 1 ]; then
            echocolor "CMake OpenCV failed, please check the external libraries that opencv depends on." "red"
            exit
        fi

        make -j $(nproc) ..
        if [ $? -eq 1 ]; then
            echocolor "Make OpenCV failed, please check the error message." "red"
            exit
        fi
        make -j $(nproc) install
        if [ $? -eq 1 ]; then
            echocolor "Install OpenCV failed, please check the error message." "red"
            exit
        else
            echocolor "Install OpenCV Success, Install Address at /usr." "red"
        fi
    else
        if version_lt $opencvVersion $version ; then
            echocolor "OpenCV already installed. Version: $opencvVersion" "yellow"
        else
            echocolor "OpenCV already installed. Version: $opencvVersion" "yellow"
        fi
        echocolor "OpenCV already installed. Version: $opencvVersion" "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallProtobuf
##############################################
function InstallProtobuf() {
    echo
    echocolor "Install Protobuf" "purple"

    echocolor "Do you want to install Protobuf? (Y/N)" "blue"
    read protobufInstall
    if [ "$protobufInstall" != "Y" ] && [ "$protobufInstall" != "y" ];then
        echocolor "Install Protobuf Failed."  "red"
        return
    fi

    #Protobuf dependencies
    InstallSimpleProgram "autoconf"
    InstallSimpleProgram "libtool"

    protoc --version
    if [ $? -eq 1 ];then
        cd ${HOME}/src

        if [ ! -f protobuf-python-3.8.0.zip ]; then
            wget https://github.com/protocolbuffers/protobuf/releases/download/v3.8.0/protobuf-python-3.8.0.zip
        fi

        if [ ! -f protoc-3.8.0-linux-aarch_64.zip ]; then
            wget https://github.com/protocolbuffers/protobuf/releases/download/v3.8.0/protoc-3.8.0-linux-aarch_64.zip
        fi

        unzip protobuf-python-3.8.0.zip
        unzip protoc-3.8.0-linux-aarch_64.zip -d protoc-3.8.0
        cp protoc-3.8.0/bin/protoc /usr/local/bin/protoc

        export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp
        cd protobuf-3.8.0/

        ./autogen.sh
        ./configure --prefix=/usr/local
        make -j $(nproc)
        make check
        make -j $(nproc) install
        ldconfig

        pip3 uninstall -y protobuf
        pip3 install Cython
        cd python/
        python3 setup.py build --cpp_implementation
        python3 setup.py test --cpp_implementation
        sudo python3 setup.py install --cpp_implementation

        echocolor "Install Protobuf Success, Install Address at /usr." "purple"
    else
        echocolor "Protobuf already Installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:Installlibtorch
##############################################
function Installlibtorch() {
    echo
    echocolor "Install libtorch" "purple"

    echocolor "Do you want to install libtorch? (Y/N)" "blue"
    read libtorchInstall
    if [ "$libtorchInstall" != "Y" ] && [ "$libtorchInstall" != "y" ];then
        echocolor "Install libtorch Failed."  "red"
        return
    fi
    
    #libtorch dependencies
    InstallSimpleProgram "python3-pip"
    InstallSimpleProgram "libopenblas-base"
    InstallSimpleProgram "libopenmpi-dev"
    InstallSimpleProgram "libopenblas-dev"
    InstallSimpleProgram "libatlas-dev"
    InstallSimpleProgram "liblapack-dev"

    torchVersion=$(python3 -c "import torch;print(torch.__version__)")
    installVersion=1.7.0

    if version_lt $torchVersion $installVersion;then
        cd ${HOME}/src

        if [ ! -f torch-1.7.0-cp36-cp36m-linux_aarch64.whl ]; then
            wget https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl
        fi

        pip3 install Cython

        pip3 install numpy torch-1.7.0-cp36-cp36m-linux_aarch64.whl
        if [ $? -eq 1 ]; then
            echocolor "Install libtorch failed." "red"
            exit
        else
            echocolor "Install libtorch Success, Install Address at ~/src/libtorch." "purple"
        fi
    else
        echocolor "libtorch already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallCvBridge
##############################################
function InstallCvBridge() {
    echo
    echocolor "Install CvBridge" "purple"

    echocolor "Do you want to install CvBridge? (Y/N)" "blue"
    read cvbridgeInstall
    if [ "$cvbridgeInstall" != "Y" ] && [ "$cvbridgeInstall" != "y" ];then
        echocolor "Install CvBridge Failed."  "red"
        return
    fi

    #CvBridge dependencies
    InstallSimpleProgram "python3-pip"
    InstallSimpleProgram "python-catkin-tools"

    pip3 install rospkg catkin_pkg

    cvbridgeVersion=1.13.0

    python -c "import cv_bridge"
    if [ $? -eq 1 ];then
        cd ${HOME}/src

        if [ ! -f vision_opencv-$cvbridgeVersion ]; then
            wget https://github.com/ros-perception/vision_opencv/archive/refs/tags/$cvbridgeVersion.zip -O vision_opencv-$cvbridgeVersion.zip
        fi

        unzip vision_opencv-$cvbridgeVersion.zip
        if [ $? -eq 1 ]; then
            echocolor "Unzip vision_opencv failed." "red"
            exit
        fi

        mkdir -p ~/cv_bridge_workspace && cd ~/cv_bridge_workspace
        mkdir -p build src install build devel logs

        mv vision_opencv-$cvbridgeVersion ~/cv_bridge_workspace/src

        cd ~/cv_bridge_workspace

        source /opt/ros/melodic/setup.sh
        if [ $? -eq 1 ]; then
            echocolor "Source ros-melodic failed, please check if ros has been install." "red"
            exit
        fi

        catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
        catkin config --install

        catkin build cv_bridge

        echo "source ~/cv_bridge_workspace/install/setup.bash --extend" >>~/.bashrc

        # cd vision_opencv-$cvbridgeVersion/cv_bridge
        # mkdir -p build && cd build

        # cmake -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so ..
        # make 
        # make install

    else
        echocolor "CvBridge already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallCmake
##############################################
function InstallCmake() {
    echo
    echocolor "Install Cmake" "purple"

    echocolor "Do you want to install Cmake? (Y/N)" "blue"
    read cmakeInstall
    if [ "$cmakeInstall" != "Y" ] && [ "$cmakeInstall" != "y" ];then
        echocolor "Install Cmake Failed."  "red"
        return
    fi

    version=3.20
    build=3
    cmakeVersion=$(cmake --version | awk 'NR==1 {print $3}')

    if version_lt $cmakeVersion $version.$build; then
        #apt remove cmake
        if [ $? -eq 1 ]; then
            echocolor "Remove older version of cmake failed." "red"
            exit
        else
            echocolor "Remove older version of cmake succeed." "green"
        fi

        cd ${HOME}/src

        if [ ! -f cmake-$version.$build.tar.gz ]; then
            wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
        fi

        tar -xzvf cmake-$version.$build.tar.gz
        if [ $? -eq 1 ]; then
            echocolor "Unzip cmake failed." "red"
            exit
        else
            echocolor "Unzip cmake succeed." "green"
        fi

        cd cmake-$version.$build
        ./bootstrap

        make -j $(nproc)
        if [ $? -eq 1 ]; then
            echocolor "Make cmake failed." "red"
            exit
        else
            echocolor "Make cmake succeed." "green"
        fi

        make -j $(nproc) install
        if [ $? -eq 1 ]; then
            echocolor "Install cmake failed." "red"
            exit
        else
            echocolor "Install cmake succeed." "green"
        fi
        
        ln -s /usr/local/bin/cmake /usr/bin/cmake
        echocolor "Install Cmake Success." "purple"        
    else
        echocolor "Cmake already install." "yellow"
    fi
    
    echo
}

##############################################
# FunctionName:InstallPyCuda
##############################################
function InstallPyCuda() {
    echo
    echocolor "Install PyCuda" "purple"

    echocolor "Do you want to install PyCuda? (Y/N)" "blue"
    read pycudaInstall
    if [ "$pycudaInstall" != "Y" ] && [ "$pycudaInstall" != "y" ];then
        echocolor "Install PyCuda Failed."  "red"
        return
    fi

    #PyCuda dependencies
    InstallSimpleProgram "build-essential"
    InstallSimpleProgram "python3-dev"
    InstallSimpleProgram "libboost-python-dev"
    InstallSimpleProgram "libboost-thread-dev"
    InstallSimpleProgram "ros-melodic-vision-msgs"

    pip3 install setuptools
    pip3 install onnx==1.4.1

    #secUser=$(who am i | awk '{print $1}')
    nvccPath=$(which nvcc)
    ln -s $nvccPath /sbin/nvcc

    # if ! which nvcc > /dev/null; then
    #     echo "ERROR: nvcc not found"
    #     exit
    # fi

    python3 -c "import pycuda; print('pycuda version:', pycuda.VERSION)"
    if [ $? -eq 1 ];then
        arch=$(uname -m)
        cd ${HOME}/src

        boost_pylib=$(basename /usr/lib/${arch}-linux-gnu/libboost_python*-py3?.so)
        boost_pylibname=${boost_pylib%.so}
        boost_pyname=${boost_pylibname/lib/}

        if [ ! -f pycuda-2019.1.2.tar.gz ]; then
            wget https://files.pythonhosted.org/packages/5e/3f/5658c38579b41866ba21ee1b5020b8225cec86fe717e4b1c5c972de0a33c/pycuda-2019.1.2.tar.gz
        fi

        CPU_CORES=$(nproc)

        tar xzvf pycuda-2019.1.2.tar.gz

        cd pycuda-2019.1.2

        python3 ./configure.py --python-exe=/usr/bin/python3 --cuda-root=/usr/local/cuda --cudadrv-lib-dir=/usr/lib/${arch}-linux-gnu --boost-inc-dir=/usr/include --boost-lib-dir=/usr/lib/${arch}-linux-gnu --boost-python-libname=${boost_pyname} --boost-thread-libname=boost_thread --no-use-shipped-boost
        
        make -j$CPU_CORES
        
        python3 setup.py build
        
        sudo python3 setup.py install
    else
        echocolor "PyCuda already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:InstallJetsonInference
##############################################
function InstallJetsonInference() {
    echo
    echocolor "Install JetsonInference" "purple"

    echocolor "Do you want to install JetsonInference? (Y/N)" "blue"
    read jetsoninferenceInstall
    if [ "$jetsoninferenceInstall" != "Y" ] && [ "$jetsoninferenceInstall" != "y" ];then
        echocolor "Install JetsonInference Failed."  "red"
        return
    fi

    #JetsonInference dependencies
    InstallSimpleProgram "ros-melodic-image-transport"

    python3 -c "import 2"
    if [ $? -eq 1 ];then
        cd ${HOME}/src

        git clone --recursive https://github.com/dusty-nv/jetson-inference

        cd jetson-inference
        
        git submodule update --init

        mkdir -p build

        cd build

        cmake ../

        make -j$(nproc)

        make install

        ldconfig
    else
        echocolor "JetsonInference already installed." "yellow"
    fi

    echo
}

##############################################
# FunctionName:GetSuperUserPermission
##############################################
function GetSuperUserPermission() {
    echo
    echocolor "Get SuperUser Permission" "purple"

    sudo su
    if [ $? -eq 1 ]; then
        echocolor "Get SuperUser Permission failed." "red"
        exit
    else
        echocolor "Get SuperUser Permission succeed." "green"
    fi

    echo
}

##############################################
# FunctionName:GetCurrentVersion
##############################################
function GetCurrentVersion() {
    shellVersion="0.0.1"
    echocolor "Install Packages Shell Version: $shellVersion" "cyan"

    echo
    echocolor "Get Current Version" "purple"

    source /etc/os-release
    RELEASE=$ID
    VERSION=$VERSION_ID
    if [ "$RELEASE" == "centos" ]; then
        release="centos"
        systemPackage="yum"
    elif [ "$RELEASE" == "debian" ]; then
        release="debian"
        systemPackage="apt-get"
    elif [ "$RELEASE" == "ubuntu" ]; then
        release="ubuntu"
        systemPackage="apt"
    fi

    systempwd="/etc/systemd/system/"
    jetpackVersion=$(cat /etc/nv_tegra_release | awk '{print $2$3$4$5}')
    tensorRTVersion=$(dpkg -l | grep TensorRT | head -1 | awk '{print $3}')
    python2Version=$(python -V 2>&1 | awk '{print $2}')
    python3Version=$(python3 -V 2>&1 | awk '{print $2}')

    echocolor "System version: $release" "cyan"
    echocolor "Jetpack version: $jetpackVersion" "cyan"
    echocolor "TensorRT version: $tensorRTVersion" "cyan"
    echocolor "Python2 Version: $python2Version" "cyan"
    echocolor "Python3 Version: $python3Version" "cyan"
    echo
}

##############################################
# FunctionName:SetGitHubUser
##############################################
function SetGitHubUser() {
    echocolor "Now Set GitHub User Account." "purple"

    username=$(git config user.name)
    useremail=$(git config user.email)

    echocolor "Current name: $username email: $useremail" "cyan"

    echocolor "Do you want to use local email to generate SSH key? (Y/N/C) (Y: Use local N: Not generate C: Change User info)" "blue"
    read generateSSH
    if [ "$generateSSH" = "Y" ] || [ "$generateSSH" = "y" ]; then
        ssh-keygen -t RSA -C $useremail
        cat ~/.ssh/id_rsa.pub
        echocolor "Please add those SSH key to your GitHub Account then Enter to continue." "yellow"
        read
    elif [ "$generateSSH" = "N" ] || [ "$generateSSH" = "n" ]; then
        return
    elif [ "$generateSSH" = "C" ] || [ "$generateSSH" = "c" ]; then
        echocolor "Enter new user name: " "blue"
        read newname
        username=newname
        echocolor "Enter new user email: " "blue"
        read newemail
        useremail=newemail
        git config --global user.name "$username"
        git config --global user.email "$useremail"
        ssh-keygen -t RSA -C $useremail
        cat ~/.ssh/id_rsa.pub
        echocolor "Please add those SSH key to your GitHub Account then Enter to continue." "yellow"
        read
    fi
}

##############################################
# FunctionName:InstallRequirements
##############################################
function InstallRequirements() {
    echo
    echocolor "Install Requirements" "purple"

    echocolor "Do you want to start install requirements? (Y/N)" "blue"
    read startRequirements
    if [[ "$startRequirements" != "Y" ]] && [[ "$startRequirements" != "y" ]]; then
        echocolor "Install requirements failed!" "red"
        exit 0
    fi

    mkdir -p ${HOME}/src

    SetGitHubUser

    UpdateSource

    InstallSimpleProgram "gcc"
    InstallSimpleProgram "git"
    InstallSimpleProgram "curl"
    InstallSimpleProgram "wget"
    InstallSimpleProgram "cmake"
    InstallSimpleProgram "vim"
    InstallSimpleProgram "make"
    InstallSimpleProgram "libnss3"
    InstallSimpleProgram "tree"
    InstallSimpleProgram "openssh-server"
    InstallSimpleProgram "openssh-client"
    
    InstallCmake

    #InstallProtobuf &
    InstallProtobuf
    InstallROS
    InstallOpenCV
    Installlibtorch
    InstallCvBridge
    InstallPyCuda
    InstallJetsonInference

    echo
}

##############################################
# FunctionName:Main
##############################################
function Main() {
    # GetSuperUserPermission
    GetCurrentVersion
    InstallRequirements
}

###################################
Main
