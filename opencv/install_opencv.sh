#!/bin/bash -e

CV_VER="3.4.5"

withcuda=0

# 安装基本的依赖项：
sudo apt install -y build-essential cmake pkg-config
# 安装和图像相关的库：
sudo apt install -y libjpeg-dev libtiff5-dev libpng-dev
# 安装视频IO包：
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libv4l-0
sudo apt install -y libxvidcore-dev libx264-dev ffmpeg libgstreamer1.0-dev libgstreamer-plugins-base1.0
# 安装highgui相关的依赖库：
sudo apt install -y libgtk2.0-dev libgtk-3-dev qt5-default
# 安装opencv进阶依赖库，操作矩阵等：
sudo apt install -y libatlas-base-dev gfortran libtbb2
# 其它一些库
sudo apt install -y libgtkglext1-dev v4l-utils
sudo apt install -y libprotobuf-dev protobuf-compiler
sudo apt install -y libgoogle-glog-dev libgflags-dev
sudo apt install -y libgphoto2-dev libeigen3-dev libhdf5-dev doxygen


# -------正式开始准备opencv和opencv_contrib---------

# 下载OpenCV
if [ -e opencv-${CV_VER}.zip ]; then
    echo -e "\033[36m opencv-${CV_VER}.zip Downloaded!!!!!! \033[0m"
else
    echo -e "\033[36m Staring Download opencv-${CV_VER}.zip \033[0m"
    wget -O opencv-${CV_VER}.zip https://github.com/Itseez/opencv/archive/${CV_VER}.zip 
fi
# 解压OpenCV
unzip -o opencv-${CV_VER}.zip

# 下载OpenCV_contrib库：
if [ -e opencv_contrib-${CV_VER}.zip ]; then
    echo -e "\033[36m opencv_contrib-${CV_VER}.zip Downloaded!!!!!! \033[0m"
else
    echo -e "\033[36m Staring Download opencv_contrib-${CV_VER}.zip \033[0m"
    wget -O opencv_contrib-${CV_VER}.zip https://github.com/Itseez/opencv_contrib/archive/${CV_VER}.zip
fi
# 解压OpenCV_contrib库：
unzip -o opencv_contrib-${CV_VER}.zip

# --------编译和安装OpenCV---------
# 进入opencv目录：
cd opencv-${CV_VER}/
# 新建build文件夹：
mkdir -p release
# 进入build文件夹：
cd release
# 配置cmake：
if [ ${withcuda} -eq 0 ]; then
	sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D ENABLE_CXX11=ON \
	-D BUILD_WITH_DEBUG_INFO=OFF \
	-D WITH_TBB=ON \
	-D WITH_CUDA=OFF \
	-D WITH_OPENCL=OFF \
	-D BUILD_DOCS=OFF \
	-D BUILD_EXAMPLES=OFF \
	-D BUILD_PERF_TESTS=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${CV_VER}/modules \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D BUILD_SHARED_LIBS=ON \
	-D WITH_GTK=ON \
	-D WITH_V4L=ON \
	-D WITH_LIBV4L=ON \
	-D WITH_OPENGL=ON \
	-D WITH_FFMPEG=ON \
	-D ENABLE_NEON=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON ..
else
	sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D ENABLE_CXX11=ON \
	-D BUILD_WITH_DEBUG_INFO=OFF \
	-D WITH_TBB=ON \
	-D WITH_CUDA=ON \
	-D WITH_OPENCL=ON \
	-D WITH_CUBLAS=ON \
	-D BUILD_DOCS=OFF \
	-D BUILD_EXAMPLES=OFF \
	-D BUILD_PERF_TESTS=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${CV_VER}/modules \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D BUILD_SHARED_LIBS=ON \
	-D WITH_GTK=ON \
	-D WITH_V4L=ON \
	-D WITH_LIBV4L=ON \
	-D WITH_OPENGL=ON \
	-D WITH_FFMPEG=ON \	
	-D OPENCV_GENERATE_PKGCONFIG=ON ..
fi
#	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${CV_VER}/modules \
#-D CMAKE_C_COMPILER=/usr/bin/gcc-5 \
#-D ENABLE_NEON=ON \
# 编译：
sudo make -j4
# 安装：
#sudo make install
#echo '/usr/local/lib' | sudo tee -a /etc/ld.so.conf.d/opencv.conf
sudo ldconfig
#printf '# OpenCV\nPKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig\nexport PKG_CONFIG_PATH\n' >> ~/.bashrc
#source ~/.bashrc
pkg-config --modversion opencv
