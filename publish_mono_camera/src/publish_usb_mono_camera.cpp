#include <ros/ros.h>
#include "iostream"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "camera_info_manager/camera_info_manager.h"

int main(int argc, char **argv)
{
  std::cout << "USB 相机程序启动..." << std::endl;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

  // cv::VideoCapture cap(0,cv::CAP_V4L2);
  cv::VideoCapture cap("/dev/video0");
  if (!cap.isOpened())
  {
    std::cout << "无法打开相机！" << std::endl;
    return -1;
  }
  std::cout << "相机初始化与启动.." << std::endl;
#if 1
  double fps = cap.get(cv::CAP_PROP_FPS);
  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH); // 分辨率;
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
#else
  double fps = 30;
  int width = 1280; // 分辨率;
  int height = 720;
  cap.set(cv::CAP_PROP_FPS, fps);           // 帧率
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width); // 分辨率
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
#endif
  cv::Mat raw_img;
  std::cout << "开始发布图像数据..." << std::endl;
  std::cout << "相机分辨率:" << width << "*" << height << std::endl;
  std::cout << "相机帧率:" << fps << "fps" << std::endl;

  while (nh.ok())
  {
    cap >> raw_img; // 获取当前帧图像

    cv::cvtColor(raw_img, raw_img, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", raw_img).toImageMsg();

    // ros::Rate loop_rate(2400);
    pub.publish(msg);

    // loop_rate.sleep();
    // if (cv::waitKey(20) == 'q' || cv::waitKey(20) == 'Q')
    // {
    //   break;
    // }
  }
}
