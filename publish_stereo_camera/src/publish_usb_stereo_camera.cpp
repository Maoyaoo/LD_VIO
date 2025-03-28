#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_camera_node");
  ros::NodeHandle nh("~"); // 使用私有命名空间获取参数

  // 从参数服务器获取参数
  std::string video_device;
  int frame_width, frame_height, fps;
  std::string left_topic, right_topic;
  bool publish_gray;

  nh.param<std::string>("video_device", video_device, "/dev/video0");
  nh.param<int>("frame_width", frame_width, 640);
  nh.param<int>("frame_height", frame_height, 480);
  nh.param<int>("fps", fps, 30);
  nh.param<std::string>("left_topic", left_topic, "/camera/left/image_raw");
  nh.param<std::string>("right_topic", right_topic, "/camera/right/image_raw");
  nh.param<bool>("publish_gray", publish_gray, false);

  // 初始化相机
  cv::VideoCapture cap(0, cv::CAP_V4L2);
  if (!cap.isOpened())
  {
    ROS_ERROR("Failed to open camera device: %s", video_device.c_str());
    return -1;
  }

  // 设置相机参数
  cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width * 2); // 双目相机是拼接的
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  // cap.set(cv::CAP_PROP_FPS, fps);

  ROS_INFO("USB stereo camera initialized successfully");
  ROS_INFO("Device: %s", video_device.c_str());
  ROS_INFO("Resolution: %dx%d", frame_width, frame_height);
  ROS_INFO("Frame rate: %d fps", (int)cap.get(cv::CAP_PROP_FPS));

  // 初始化图像发布
  // image_transport::ImageTransport it(nh);
  // image_transport::Publisher pub_left = it.advertise(left_topic, 1);
  // image_transport::Publisher pub_right = it.advertise(right_topic, 1);

  // 替换 image_transport 代码：
ros::Publisher pub_left = nh.advertise<sensor_msgs::Image>(left_topic, 1);
ros::Publisher pub_right = nh.advertise<sensor_msgs::Image>(right_topic, 1);

  cv::Mat stereo_img, imLeft, imRight;
  // ros::Rate loop_rate(fps);

  while (ros::ok())
  {
    cap >> stereo_img;
    if (stereo_img.empty())
    {
      ROS_WARN("Received empty frame, camera may be disconnected");
      continue;
    }

    // 分割左右图像
    imLeft = stereo_img.colRange(0, frame_width).clone();
    imRight = stereo_img.colRange(frame_width, stereo_img.cols).clone();

    if (publish_gray)
    {
      cv::cvtColor(imLeft, imLeft, cv::COLOR_BGR2GRAY);
      cv::cvtColor(imRight, imRight, cv::COLOR_BGR2GRAY);
    }

    // 发布图像
    auto msg_left = cv_bridge::CvImage(
                        std_msgs::Header(),
                        publish_gray ? "mono8" : "bgr8",
                        imLeft)
                        .toImageMsg();

    auto msg_right = cv_bridge::CvImage(
                         std_msgs::Header(),
                         publish_gray ? "mono8" : "bgr8",
                         imRight)
                         .toImageMsg();

    msg_left->header.stamp = ros::Time::now();
    msg_right->header.stamp = msg_left->header.stamp;

    pub_left.publish(msg_left);
    pub_right.publish(msg_right);

    // loop_rate.sleep();
  }

  cap.release();
  return 0;
}
