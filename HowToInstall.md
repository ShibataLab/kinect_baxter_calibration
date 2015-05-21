# How To Install Kinect V2 Driver in Ubuntu
It is still being edited.(編集中)\\
This page contains the install instructions for setting up KinectV2 with Ubuntu 15.04 Operating Systems.

##参考
IAI Kinect2 : https://github.com/code-iai/iai_kinect2/blob/master/README.md

##最初に
CMakeLists.txtを変更する必要がある。
48行目～50行目を
```
# LibUSB
GET_FILENAME_COMPONENT(LIBUSB_DIR "${MY_DIR}/../../depends/libusb/" REALPATH)

INCLUDE_DIRECTORIES("${LIBUSB_DIR}/include/libusb-1.0/")
LINK_DIRECTORIES("${LIBUSB_DIR}/lib/")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
```
と変更する。


##インストール
libfructions2をインストールする。
```
cd ~
sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so
git clone https://github.com/OpenKinect/libfreenect2
```
インストールを行うか聞かれるので、Y　を入力する。その後
```
cd libfreenect2/depends
./install_ubuntu.sh
```
を行い、現在いるディレクトリの一つ上のディレクトリに" build "ディレクトリを作成していく。
```
cd ..
mkdir build
cd build
mkdir linux
cd linux
cmake ../../examples/protonect/ -DENABLE_CXX11=ON
make && sudo make install
```

このレポジトリのクローンを、自分のcatkinワークスペースにインストールし構築する。
```
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```

##root権限にしなくても実行できるようにする

" 90-kinect2.rules "という名前のファイルを、" /etc/udev/rules.d/ "の中に作成。そのファイルの中に
```
# ATTR{product}=="Kinect2"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"
```
を書く。

##Kinectを起動する
