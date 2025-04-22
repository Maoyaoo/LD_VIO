#ifndef __DEPTH_TO_POINTCLOUND__
#define __DEPTH_TO_POINTCLOUND__

#include <opencv2/core.hpp>
#include <Eigen/Core> // Eigen::Vector3d (核心模块)
#include <vector>

class depth_to_pointcloud
{
private:
    /* camera parameters */
    double cx_, cy_, fx_, fy_;

    /*外参*/
    Eigen::Matrix4d cam2body_;

    /* depth image projection filtering */
    bool has_first_depth_;
    double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
    int depth_filter_margin_;
    bool use_depth_filter_;
    double k_depth_scaling_factor_;
    int skip_pixel_;

    double min_ray_length_, max_ray_length_; // range of doing raycasting

public:
    depth_to_pointcloud(double cx, double cy, double fx, double fy);
    ~depth_to_pointcloud();

    std::vector<Eigen::Vector3d> to_cam_pointclound(cv::Mat &depth_image);
    std::vector<Eigen::Vector3d> to_world_pointclound(cv::Mat &depth_image, Eigen::Matrix3d &odom_r, Eigen::Vector3d &odom_t);
};

#endif