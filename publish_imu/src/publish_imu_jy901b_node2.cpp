#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <map>

// Global variables
std::map<int, uint8_t> buff;
int key = 0;
std::vector<double> angularVelocity(3, 0);
std::vector<double> acceleration(3, 0);
std::vector<double> magnetometer(3, 0);
std::vector<double> angle_degree(3, 0);

// Publishers
ros::Publisher imu_pub;
ros::Publisher mag_pub;

bool checkSum(const std::vector<uint8_t> &data, uint8_t check_data)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < data.size(); i++)
    {
        sum += data[i];
    }
    return (sum & 0xff) == check_data;
}

std::vector<int16_t> hexToShort(const std::vector<uint8_t> &raw_data)
{
    std::vector<int16_t> result(4, 0);
    for (size_t i = 0; i < 4; i++)
    {
        result[i] = (raw_data[i * 2 + 1] << 8) | raw_data[i * 2];
    }
    return result;
}

void handleSerialData(uint8_t raw_data)
{
    bool angle_flag = false;

    buff[key] = raw_data;
    key++;

    if (buff[0] != 0x55)
    {
        key = 0;
        buff.clear();
        return;
    }

    if (key < 11)
    {
        return;
    }

    std::vector<uint8_t> data_buff;
    for (const auto &pair : buff)
    {
        data_buff.push_back(pair.second);
    }

    if (buff[1] == 0x51)
    {
        if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
        {
            auto accel = hexToShort(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
            for (int i = 0; i < 3; i++)
            {
                acceleration[i] = accel[i] / 32768.0 * 16 * 9.8;
            }
        }
        else
        {
            ROS_WARN("0x51 Check failure");
        }
    }
    else if (buff[1] == 0x52)
    {
        if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
        {
            auto gyro = hexToShort(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
            for (int i = 0; i < 3; i++)
            {
                angularVelocity[i] = gyro[i] / 32768.0 * 2000 * M_PI / 180;
            }
        }
        else
        {
            ROS_WARN("0x52 Check failure");
        }
    }
    else if (buff[1] == 0x53)
    {
        if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
        {
            auto angle = hexToShort(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
            for (int i = 0; i < 3; i++)
            {
                angle_degree[i] = angle[i] / 32768.0 * 180;
            }
            angle_flag = true;
        }
        else
        {
            ROS_WARN("0x53 Check failure");
        }
    }
    else if (buff[1] == 0x54)
    {
        if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
        {
            auto mag = hexToShort(std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 10));
            for (int i = 0; i < 3; i++)
            {
                magnetometer[i] = mag[i];
            }
        }
        else
        {
            ROS_WARN("0x54 Check failure");
        }
    }
    else
    {
        buff.clear();
        key = 0;
        return;
    }

    buff.clear();
    key = 0;

    if (angle_flag)
    {
        ros::Time stamp = ros::Time::now();

        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;

        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "base_link";

        mag_msg.header.stamp = stamp;
        mag_msg.header.frame_id = "base_link";

        // Convert Euler angles to quaternion
        tf2::Quaternion quat;
        quat.setRPY(
            angle_degree[0] * M_PI / 180,
            angle_degree[1] * M_PI / 180,
            angle_degree[2] * M_PI / 180);

        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        imu_msg.angular_velocity.x = angularVelocity[0];
        imu_msg.angular_velocity.y = angularVelocity[1];
        imu_msg.angular_velocity.z = angularVelocity[2];

        imu_msg.linear_acceleration.x = acceleration[0];
        imu_msg.linear_acceleration.y = acceleration[1];
        imu_msg.linear_acceleration.z = acceleration[2];

        mag_msg.magnetic_field.x = magnetometer[0];
        mag_msg.magnetic_field.y = magnetometer[1];
        mag_msg.magnetic_field.z = magnetometer[2];

        imu_pub.publish(imu_msg);
        mag_pub.publish(mag_msg); // 磁力计
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("~");

    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baud", baudrate, 9600);

    ROS_INFO("IMU Type: Normal Port:%s baud:%d", port.c_str(), baudrate);

    serial::Serial wt_imu;
    try
    {
        wt_imu.setPort(port);
        wt_imu.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
        wt_imu.setTimeout(timeout);
        wt_imu.open();
        ROS_INFO("\033[32mSerial port opened successfully...\033[0m");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("\033[31mSerial port opening failure: %s\033[0m", e.what());
        return -1;
    }

    imu_pub = nh.advertise<sensor_msgs::Imu>("/wit/imu", 10);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("/wit/mag", 10);

    while (ros::ok())
    {
        try
        {
            size_t buff_count = wt_imu.available();
            if (buff_count > 0)
            {
                std::vector<uint8_t> buff_data;
                wt_imu.read(buff_data, buff_count);
                for (size_t i = 0; i < buff_count; i++)
                {
                    handleSerialData(buff_data[i]);
                }
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("exception: %s", e.what());
            ROS_ERROR("imu disconnect");
            return -1;
        }

        ros::spinOnce();
    }

    return 0;
}
