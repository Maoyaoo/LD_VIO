# 安装依赖
```
sudo apt-get update
sudo apt-get install libv4l-dev
sudo apt-get install ros-$ROS_DISTRO-serial
sudo apt install ros-$ROS_DISTRO-rviz-imu-plugin

```

# imu设备绑定
```
sudo bash bind_usb.sh
```

# 发布IMU话题
```
roslaunch publish_imu jy901b_node2_imu_driver.launch
```

# 注意事项
虚拟机打开摄像头要设置虚拟机兼容usb3
解决方法：usb兼容性选USB3.1