# Get strated
```
roscd;cd ..
cd src
git clone https://github.com/luppyfox/Autonomous_Luggage_Companion.git
```

# 2. Install additional package
for installing package please follow this step:
```
roscd;cd ..
cd src/Autonomous_Luggage_Companion
```
## 2.1 Camera
```
git clone https://github.com/orbbec/ros_astra_camera.git
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
sudo apt install ros-$ROS_DISTRO-rgbd-launch libuvc-dev
```
## 2.2 Audio
```
https://github.com/ros-drivers/audio_common.git
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```
