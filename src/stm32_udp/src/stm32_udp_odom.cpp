#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 2000

class UdpOdomNode : public rclcpp::Node
{
public:
    UdpOdomNode() : Node("stm32_udp_odom")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        memset(&addr, 0, sizeof(addr));

        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT);
        addr.sin_addr.s_addr = INADDR_ANY;

        bind(sockfd, (const struct sockaddr *)&addr, sizeof(addr));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&UdpOdomNode::recv_loop, this));

        RCLCPP_INFO(this->get_logger(), "STM32 UDP Odom Node Started");
    }

private:
    void recv_loop()
    {
        char buffer[128];

        socklen_t len = sizeof(addr);
        int bufferLen = recvfrom(sockfd, (char *)buffer, sizeof(buffer), 0, (sockaddr *)&addr, &len);
        if (bufferLen < 0 || bufferLen > 20)
        {
            return;
        }
        float x, y, speed, angle, theta;

        /*
        (void)angle;
        (void)time;
        const char SPACE = ' ';
        char* xString = strtok(buffer,&SPACE);
        char* yString = strtok(NULL,&SPACE);
        char* angleString = strtok(NULL,&SPACE);
        char* timeString = strtok(NULL,&SPACE);

        x = std::stof(xString);
        y = std::stof(yString);
        angle = std::stof(angleString);
        time = std::stoi(timeString);
        */
        memcpy(&x, buffer + 0, 4);
        memcpy(&y, buffer + 4, 4);
        memcpy(&speed, buffer + 8, 4);
        memcpy(&angle, buffer + 12, 4);
        memcpy(&theta, buffer + 16, 4);

        //
        RCLCPP_INFO(this->get_logger(), "\nangle: %f\nx: %f\ny: %f\nspeed: %f\ntheta: %f", angle, x, y, speed,theta);
        //

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = q_msg;

        odom.twist.twist.linear.x = speed;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = theta;

        odom.pose.covariance = {
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2};

        odom.twist.covariance = {
            1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1};

        publisher_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = q_msg;

        tf_broadcaster_->sendTransform(t);
    }

    int sockfd;
    struct sockaddr_in addr;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpOdomNode>());
    rclcpp::shutdown();
    return 0;
}

/*
① 雷达
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200
② 你的 UDP 里程计节点
ros2 run stm32_udp stm32_udp_odom
③ 静态 TF
ros2 run tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_link laser_frame
④ SLAM
ros2 launch slam_toolbox online_async_launch.py
⑤ RViz

source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/setup.bash

Fixed Frame = map
*/