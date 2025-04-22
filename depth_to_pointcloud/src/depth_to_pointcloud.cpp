#include "depth_to_pointcloud.h"

#include <iostream>

depth_to_pointcloud::depth_to_pointcloud(double cx, double cy, double fx, double fy)
    : cx_(cx), cy_(cy), fx_(fx), fy_(fy)
{
    use_depth_filter_ = true;
    depth_filter_tolerance_ = 0.15;
    depth_filter_maxdist_ = 5.0;
    depth_filter_mindist_ = 0.2;
    depth_filter_margin_ = 2;
    k_depth_scaling_factor_ = 1000.0;
    skip_pixel_ = 2;

    min_ray_length_ = 0.3;
    max_ray_length_ = 5.0;
}

depth_to_pointcloud::~depth_to_pointcloud()
{

}

std::vector<Eigen::Vector3d> depth_to_pointcloud::to_cam_pointclound(cv::Mat &depth_image)
{
    std::vector<Eigen::Vector3d> proj_points;
    int proj_points_cnt = 0; // 重置投影点计数器（不清空vector，复用内存）

    uint16_t *row_ptr;
    int cols = depth_image.cols;                      // 深度图像宽度（像素）
    int rows = depth_image.rows;                      // 深度图像高度（像素）
    int skip_pix = skip_pixel_ > 0 ? skip_pixel_ : 1; // 像素跳过的步长（降低计算量）

    // 预先分配足够空间(上取整)
    int points_per_row = (cols + skip_pix - 1) / skip_pix;
    int num_rows = (rows + skip_pix - 1) / skip_pix;
    int expected_points = points_per_row * num_rows;
    proj_points.reserve(expected_points);

    double depth; // 临时存储深度值
    Eigen::Vector3d pt_cam, pt_world;

    const double inv_fx = 1.0 / fx_;
    const double inv_fy = 1.0 / fy_;
    const double inv_depth_factor = 1.0 / k_depth_scaling_factor_;

    // ========================= 未启用深度过滤器 =========================
    if (!use_depth_filter_)
    {
        for (int v = 0; v < rows; v += skip_pix) // 遍历图像行（跳过指定步长）
        {
            row_ptr = depth_image.ptr<uint16_t>(v); // 当前行指针

            for (int u = 0; u < cols; u += skip_pix) // 遍历图像列
            {
                Eigen::Vector3d proj_pt;
                // 1. 读取深度值并转换为米单位（假设原始数据为uint16毫米级）
                depth = (*row_ptr) * inv_depth_factor; // 读取并缩放深度值
                row_ptr += skip_pix;                   // 移动指针到下一个像素

                // 2. 将像素坐标(u,v)转换为相机坐标系下的3D点（透视投影模型）
                pt_cam(0) = (u - cx_) * depth * inv_fx; // X = (u - cx) * depth / fx
                pt_cam(1) = (v - cy_) * depth * inv_fy; // Y = (v - cy) * depth / fy
                pt_cam(2) = depth;                      // Z = depth

                // （调试：打印图像中心点的深度值）
                if (u == 320 && v == 240)
                    std::cout << "depth: " << depth << std::endl;

                // 4. 存储投影后的3D点
                proj_points[proj_points_cnt++] = pt_cam;
            }
        }
    }
    // ========================= 启用深度过滤器 =========================
    else
    {
        // 标记是否为第一帧（跳过第一帧的时序检查）
        if (!has_first_depth_)
            has_first_depth_ = true;
        else
        {
            // 遍历图像（跳过边缘区域 margin 和指定步长）
            for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pix)
            {
                row_ptr = depth_image.ptr<uint16_t>(v) + depth_filter_margin_;

                for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pix)
                {
                    depth = (*row_ptr) * inv_depth_factor; // 读取并缩放深度值
                    row_ptr += skip_pix;                   // 移动指针到下一个像素

                    // ------------------------- 深度值过滤 -------------------------
                    // 情况1：无效深度（0值）
                    if (*row_ptr == 0)
                        depth = max_ray_length_ + 0.1; // 设为最大距离外（标记为无效）
                    // 情况2：深度过近
                    else if (depth < depth_filter_mindist_)
                        continue; // 跳过该点
                    // 情况3：深度过远
                    else if (depth > depth_filter_maxdist_)
                        depth = max_ray_length_ + 0.1; // 设为最大距离外

                    // ------------------------- 投影到世界坐标系 -------------------------
                    pt_cam(0) = (u - cx_) * depth * inv_fx; // 相机坐标系X
                    pt_cam(1) = (v - cy_) * depth * inv_fy; // 相机坐标系Y
                    pt_cam(2) = depth;                      // 相机坐标系Z

                    // 存储有效点
                    proj_points[proj_points_cnt++] = pt_cam;
                }
            }
        }
    }
}

std::vector<Eigen::Vector3d> depth_to_pointcloud::to_world_pointclound(cv::Mat &depth_image, Eigen::Matrix3d &odom_r, Eigen::Vector3d &odom_t)
{
    // ====================== 1. 从里程计获取相机位姿 ======================

    // 构建车身到世界坐标系的变换矩阵（4x4齐次矩阵）
    Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
    body2world.block<3, 3>(0, 0) = odom_r;
    body2world.block<3, 1>(0, 3) = odom_t;

    // 计算相机到世界坐标系的变换：T_world_cam = T_world_body * T_body_cam
    Eigen::Matrix4d cam2world_T = body2world * cam2body_;

    // 相机在世界坐标系下的位置和旋转矩阵
    Eigen::Vector3d cam2world_t = cam2world_T.block<3, 1>(0, 3);
    Eigen::Matrix3d cam2world_r = cam2world_T.block<3, 3>(0, 0);

    std::vector<Eigen::Vector3d> proj_points;
    int proj_points_cnt = 0; // 重置投影点计数器（不清空vector，复用内存）

    uint16_t *row_ptr;
    int cols = depth_image.cols;                      // 深度图像宽度（像素）
    int rows = depth_image.rows;                      // 深度图像高度（像素）
    int skip_pix = skip_pixel_ > 0 ? skip_pixel_ : 1; // 像素跳过的步长（降低计算量）

    // 预先分配足够空间(上取整)
    int points_per_row = (cols + skip_pix - 1) / skip_pix;
    int num_rows = (rows + skip_pix - 1) / skip_pix;
    int expected_points = points_per_row * num_rows;
    proj_points.reserve(expected_points);

    double depth; // 临时存储深度值
    Eigen::Vector3d pt_cam, pt_world;

    const double inv_fx = 1.0 / fx_;
    const double inv_fy = 1.0 / fy_;
    const double inv_depth_factor = 1.0 / k_depth_scaling_factor_;

    // ========================= 未启用深度过滤器 =========================
    if (!use_depth_filter_)
    {
        for (int v = 0; v < rows; v += skip_pix) // 遍历图像行（跳过指定步长）
        {
            row_ptr = depth_image.ptr<uint16_t>(v); // 当前行指针

            for (int u = 0; u < cols; u += skip_pix) // 遍历图像列
            {
                Eigen::Vector3d proj_pt;
                // 1. 读取深度值并转换为米单位（假设原始数据为uint16毫米级）
                depth = (*row_ptr) * inv_depth_factor; // 读取并缩放深度值
                row_ptr += skip_pix;                   // 移动指针到下一个像素

                // 2. 将像素坐标(u,v)转换为相机坐标系下的3D点（透视投影模型）
                pt_cam(0) = (u - cx_) * depth * inv_fx; // X = (u - cx) * depth / fx
                pt_cam(1) = (v - cy_) * depth * inv_fy; // Y = (v - cy) * depth / fy
                pt_cam(2) = depth;                      // Z = depth

                // 3. 转换到世界坐标系：旋转 + 平移
                pt_world = cam2world_r * pt_cam + cam2world_t;

                // （调试：打印图像中心点的深度值）
                if (u == 320 && v == 240)
                    std::cout << "depth: " << depth << std::endl;

                // 4. 存储投影后的3D点
                proj_points[proj_points_cnt++] = pt_world;
            }
        }
    }
    // ========================= 启用深度过滤器 =========================
    else
    {
        // 标记是否为第一帧（跳过第一帧的时序检查）
        if (!has_first_depth_)
            has_first_depth_ = true;
        else
        {
            // 遍历图像（跳过边缘区域 margin 和指定步长）
            for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pix)
            {
                row_ptr = depth_image.ptr<uint16_t>(v) + depth_filter_margin_;

                for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pix)
                {
                    depth = (*row_ptr) * inv_depth_factor; // 读取并缩放深度值
                    row_ptr += skip_pix;                   // 移动指针到下一个像素

                    // ------------------------- 深度值过滤 -------------------------
                    // 情况1：无效深度（0值）
                    if (*row_ptr == 0)
                        depth = max_ray_length_ + 0.1; // 设为最大距离外（标记为无效）
                    // 情况2：深度过近
                    else if (depth < depth_filter_mindist_)
                        continue; // 跳过该点
                    // 情况3：深度过远
                    else if (depth > depth_filter_maxdist_)
                        depth = max_ray_length_ + 0.1; // 设为最大距离外

                    // ------------------------- 投影到世界坐标系 -------------------------
                    pt_cam(0) = (u - cx_) * depth * inv_fx; // 相机坐标系X
                    pt_cam(1) = (v - cy_) * depth * inv_fy; // 相机坐标系Y
                    pt_cam(2) = depth;                      // 相机坐标系Z

                    // 转换到世界坐标系
                    pt_world = cam2world_r * pt_cam + cam2world_t;

                    // 存储有效点
                    proj_points[proj_points_cnt++] = pt_world;
                }
            }
        }
    }
}



// todo处理nan值
#if 0
// 遍历图像每个像素进行深度值处理 ---------------
for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
{
  float raw = raw_data[index]; // 原始深度值

  // 处理特殊情况：
  // 1. 如果原始值是NaN（无效数据），替换为maxDepthValue
  // 2. 限制深度值在[minDepthValue, maxDepthValue]范围内
  float depth_value = std::isnan(raw) ? maxDepthValue : raw; // 处理NaN
  depth_value = std::min(depth_value, maxDepthValue);        // 上限截断
  depth_value = std::max(depth_value, minDepthValue);        // 下限截断

  depth_data[index] = depth_value; // 存储处理后的值
}


#endif

int main()
{

    
}