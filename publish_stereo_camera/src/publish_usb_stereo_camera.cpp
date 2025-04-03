#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_camera_node");
  ros::NodeHandle nh("~");

  // 参数获取
  std::string video_device;
  int frame_width, frame_height, out_width, out_height, fps;
  std::string left_topic, right_topic;
  bool publish_gray;
  std::string frame_id;

  nh.param<std::string>("video_device", video_device, "/dev/video0");
  nh.param<int>("frame_width", frame_width, 1280);
  nh.param<int>("frame_height", frame_height, 720);
  nh.param<int>("out_width", out_width, 640);
  nh.param<int>("out_height", out_height, 480);
  nh.param<int>("fps", fps, 30);
  nh.param<std::string>("left_topic", left_topic, "/camera/left/image_raw");
  nh.param<std::string>("right_topic", right_topic, "/camera/right/image_raw");
  nh.param<bool>("publish_gray", publish_gray, false);
  nh.param<std::string>("frame_id", frame_id, "camera");

  // 初始化相机
  cv::VideoCapture cap(video_device, cv::CAP_V4L2);
  if (!cap.isOpened())
  {
    ROS_ERROR("Failed to open camera: %s", video_device.c_str());
    return -1;
  }

  // 设置相机参数
  cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width * 2);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
  cap.set(cv::CAP_PROP_FPS, fps);
  cap.set(cv::CAP_PROP_BUFFERSIZE, 2); // 减少缓冲区积压

  // 验证实际参数
  double actual_fps = cap.get(cv::CAP_PROP_FPS);
  if (actual_fps <= 30)
  {
    ROS_WARN("Actual FPS is %d, using the default FPS: %d", actual_fps,fps);
    actual_fps = fps;
  }
  ROS_INFO("Running at %.1f fps | Resolution: %dx%d",
           actual_fps,
           (int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
           (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT));

  // 初始化图像传输
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left = it.advertise(left_topic, 1);
  image_transport::Publisher pub_right = it.advertise(right_topic, 1);

  cv::Mat stereo_img;
  ros::Rate rate(actual_fps);

  while (ros::ok())
  {
    ros::Time acquisition_time = ros::Time::now(); // 采集前获取时间戳

    cap >> stereo_img;
    if (stereo_img.empty())
    {
      ROS_WARN_THROTTLE(5, "Empty frame received");
      continue;
    }

    // 缩放图像
    cv::resize(stereo_img, stereo_img, cv::Size(out_width * 2, out_height), 0, 0, cv::INTER_LINEAR);

    // 分割左右图像（无拷贝操作）
    cv::Mat left_img = stereo_img(cv::Rect(0, 0, out_width, out_height));
    cv::Mat right_img = stereo_img(cv::Rect(out_width, 0, out_width, out_height));

    // 灰度转换（按需进行）
    if (publish_gray)
    {
      cv::cvtColor(left_img, left_img, cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_img, right_img, cv::COLOR_BGR2GRAY);
    }

    // 创建消息并填充
    std_msgs::Header header;
    header.stamp = acquisition_time; // 使用采集时的时间戳
    header.frame_id = frame_id;

    auto left_msg = cv_bridge::CvImage(header, publish_gray ? "mono8" : "bgr8", left_img).toImageMsg();
    auto right_msg = cv_bridge::CvImage(header, publish_gray ? "mono8" : "bgr8", right_img).toImageMsg();

    // 零拷贝发布
    if (pub_left.getNumSubscribers() > 0)
      pub_left.publish(left_msg);
    if (pub_right.getNumSubscribers() > 0)
      pub_right.publish(right_msg);

    ros::spinOnce();
    rate.sleep();
  }

  cap.release();
  return 0;
}
