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
  std::cout << "程序启动..." << std::endl;
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_L = it.advertise("camera/left/image_raw", 1);
  image_transport::Publisher pub_R = it.advertise("camera/right/image_raw", 1);
  cv::Mat imLeft, imRight, imLeftRect, imRightRect;

  
  cv::VideoCapture cap(0,cv::CAP_V4L2);
//   cv::VideoCapture cap("rtsp://192.168.1.21:555/live");
  if (!cap.isOpened())
  {
    std::cout << "无法打开相机！" << std::endl;
    return -1;
  }
  std::cout << "USB双目相机初始化与启动.." << std::endl;
  double fps = cap.get(cv::CAP_PROP_FPS);
  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH); // 分辨率;
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

  // cap.set(cv::CAP_PROP_FRAME_WIDTH, width); // 分辨率
  // cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  // cap.set(cv::CAP_PROP_FPS, fps); // 帧率

  cv::Mat stereo_img;
  std::cout << "开始发布图像数据..." << std::endl;
  std::cout << "相机分辨率:" << width * 0.5 << "*" << height << std::endl;
  std::cout << "相机帧率:" << fps << "fps" << std::endl;

  while (nh.ok())
  {
    cap >> stereo_img; // 获取当前帧图像
    imLeft = stereo_img.colRange(0, width / 2).clone();
    imRight = stereo_img.colRange(width / 2, width).clone();
    cv::cvtColor(imLeft, imLeft, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imRight, imRight, cv::COLOR_BGR2GRAY);

    sensor_msgs::ImagePtr msg_L = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
    sensor_msgs::ImagePtr msg_R = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();

    ros::Rate loop_rate(240);
    pub_L.publish(msg_L);
    pub_R.publish(msg_R);
    loop_rate.sleep();
    if (cv::waitKey(20) == 'q' || cv::waitKey(20) == 'Q')
    {
      break;
    }
  }
}