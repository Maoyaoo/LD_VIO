#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread> // 添加这个头文件
#include <atomic>
#include <vector>
#include <mutex>

#define BAUDRATE 115200  // 更改为115200波特率（更常见）

class IMUDriverNode {
public:
    IMUDriverNode() : nh_("~") {
        // 初始化参数
        nh_.param<std::string>("port_name", port_name_, "/dev/ttyUSB0");
        
        // 初始化发布者
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        
        // 启动串口线程
        imu_thread_running_ = true;
        imu_thread_ = std::thread(&IMUDriverNode::imuThread, this);
    }

    ~IMUDriverNode() {
        imu_thread_running_ = false;
        if (imu_thread_.joinable()) {
            imu_thread_.join();
        }
    }

private:
    void imuThread() {
        serial::Serial imu_serial;
        try {
            imu_serial.setPort(port_name_);
            imu_serial.setBaudrate(BAUDRATE);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            imu_serial.setTimeout(timeout);
            imu_serial.open();
            ROS_INFO("Serial port opened successfully: %s", port_name_.c_str());
        } catch (const serial::IOException& e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
            return;
        }

        std::vector<uint8_t> buff;
        while (ros::ok() && imu_thread_running_) {
            try {
                if (imu_serial.available()) {
                    uint8_t data;
                    size_t bytes_read = imu_serial.read(&data, 1);
                    if (bytes_read != 1) continue;

                    buff.push_back(data);

                    // 帧头检查
                    if (buff.size() == 1 && buff[0] != 0x55) {
                        buff.clear();
                        continue;
                    }

                    // 完整帧检查
                    if (buff.size() >= 11) {
                        processFrame(buff);
                        buff.clear();
                    }
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Serial port error: %s", e.what());
                break;
            }
        }
        imu_serial.close();
    }

    void processFrame(const std::vector<uint8_t>& frame) {
        if (frame.size() < 11) return;
        
        // 校验和检查
        uint8_t sum = 0;
        for (size_t i = 0; i < 10; ++i) {
            sum += frame[i];
        }
        if (sum != frame[10]) {
            ROS_WARN("Checksum failed for frame type: 0x%02X", frame[1]);
            return;
        }

        // 数据解析
        std::vector<uint8_t> data(frame.begin() + 2, frame.begin() + 10);
        auto short_data = hexToShort(data);

        std::lock_guard<std::mutex> lock(data_mutex_);
        
        switch (frame[1]) {
            case 0x51:  // 加速度
                for (int i = 0; i < 3; ++i) {
                    acceleration_[i] = short_data[i] / 32768.0 * 16 * 9.8;
                }
                break;
            case 0x52:  // 角速度
                for (int i = 0; i < 3; ++i) {
                    angular_velocity_[i] = short_data[i] / 32768.0 * 2000 * M_PI / 180;
                }
                break;
            case 0x53:  // 角度
                for (int i = 0; i < 3; ++i) {
                    angle_degree_[i] = short_data[i] / 32768.0 * 180;
                }
                publishIMUData();
                break;
            case 0x54:  // 磁力计
                for (int i = 0; i < 3; ++i) {
                    magnetometer_[i] = short_data[i];
                }
                break;
            default:
                ROS_WARN("Unknown frame type: 0x%02X", frame[1]);
        }
    }

    void publishIMUData() {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        // 设置方向四元数
        tf2::Quaternion quat;
        quat.setRPY(
            angle_degree_[0] * M_PI / 180,
            angle_degree_[1] * M_PI / 180,
            angle_degree_[2] * M_PI / 180
        );
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        // 设置角速度
        imu_msg.angular_velocity.x = angular_velocity_[0];
        imu_msg.angular_velocity.y = angular_velocity_[1];
        imu_msg.angular_velocity.z = angular_velocity_[2];

        // 设置线性加速度
        imu_msg.linear_acceleration.x = acceleration_[0];
        imu_msg.linear_acceleration.y = acceleration_[1];
        imu_msg.linear_acceleration.z = acceleration_[2];

        // 发布消息
        imu_pub_.publish(imu_msg);
    }

    std::vector<int16_t> hexToShort(const std::vector<uint8_t>& hex_data) {
        std::vector<int16_t> result(4, 0);
        for (size_t i = 0; i < 4; ++i) {
            result[i] = (hex_data[i*2+1] << 8) | hex_data[i*2];
        }
        return result;
    }

    // 成员变量
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    std::thread imu_thread_;
    std::atomic_bool imu_thread_running_;
    std::mutex data_mutex_;
    
    std::string port_name_;
    std::vector<double> acceleration_{0, 0, 0};
    std::vector<double> angular_velocity_{0, 0, 0};
    std::vector<double> angle_degree_{0, 0, 0};
    std::vector<double> magnetometer_{0, 0, 0};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_driver");
    IMUDriverNode node;
    ros::spin();
    return 0;
}
