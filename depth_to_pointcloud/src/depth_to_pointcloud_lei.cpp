#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>

class DepthToPointCloud
{
public:
    DepthToPointCloud() : nh_("~"), got_camera_info_(false)
    {
        // 从参数服务器获取配置参数
        nh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_rect_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/depth/camera_info");
        nh_.param<std::string>("output_topic", output_topic_, "/d435i_point_cloud");
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<int>("queue_size", queue_size_, 10);
        
        // 初始化发布者和订阅者
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size_);
        depth_sub_ = nh_.subscribe(depth_topic_, queue_size_, 
                                  &DepthToPointCloud::depthImageCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                       &DepthToPointCloud::cameraInfoCallback, this);
        
        ROS_INFO("Depth to PointCloud converter initialized");
        ROS_INFO("Subscribing to: %s and %s", depth_topic_.c_str(), camera_info_topic_.c_str());
        ROS_INFO("Publishing to: %s with frame_id %s", output_topic_.c_str(), frame_id_.c_str());
    }

private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        if (!got_camera_info_)
        {
            camera_matrix_ = msg->K;
            got_camera_info_ = true;
            ROS_INFO("Received camera intrinsics");
        }
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if (!got_camera_info_)
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for camera info...");
            return;
        }

        // 检查图像格式
        if (msg->encoding != "16UC1")
        {
            ROS_ERROR_THROTTLE(1.0, "Unsupported image encoding: %s (expected 16UC1)", 
                              msg->encoding.c_str());
            return;
        }

        // 安全地获取图像数据
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&msg->data[0]);
        const int width = msg->width;
        const int height = msg->height;
        const int total_pixels = width * height;

        // 预分配点云内存
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(total_pixels / 2);  // 假设大约一半的点是有效的

        // 获取相机内参（线程安全）
        std::array<double, 9> K;
        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            K = camera_matrix_;
        }

        const double fx = K[0];
        const double fy = K[4];
        const double cx = K[2];
        const double cy = K[5];

        // 转换深度图到点云
        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                const uint16_t depth_value = depth_data[v * width + u];
                
                // 跳过无效深度值
                if (depth_value == 0) continue;

                // 转换为米单位并计算3D坐标
                const double z = depth_value / 1000.0;
                const double x = z * (u - cx) / fx;
                const double y = z * (v - cy) / fy;

                cloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }

        // 发布点云
        if (!cloud->empty())
        {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header = msg->header;  // 保持原始时间戳
            cloud_msg.header.frame_id = frame_id_;
            point_cloud_pub_.publish(cloud_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher point_cloud_pub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;

    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string output_topic_;
    std::string frame_id_;
    int queue_size_;

    std::array<double, 9> camera_matrix_;
    bool got_camera_info_;
    std::mutex camera_info_mutex_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_pointcloud");
    DepthToPointCloud converter;
    ros::spin();
    return 0;
}
