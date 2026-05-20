#ifndef QUALISYS_ODOM_H
#define QUALISYS_ODOM_H

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Mensagens do package
#include <qualisys_ros2/msg/subject.hpp>

// KalmanFilter (igual ao ROS 1 — não muda)
#include "KalmanFilter.h"

namespace qualisys
{

class QualisysOdom : public rclcpp::Node
{
public:
    /*
     * @brief Constructor
     */
    QualisysOdom();

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

private:
    // Disable copy constructor and assign operator
    QualisysOdom(const QualisysOdom&);
    QualisysOdom& operator=(const QualisysOdom&);

    // Callback function for receiving subject msgs
    void QualisysCallback(const qualisys_ros2::msg::Subject::SharedPtr qualisys_msg);

    qualisys::KalmanFilter kf_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<qualisys_ros2::msg::Subject>::SharedPtr qualisys_sub_;
};

} // namespace qualisys

#endif // QUALISYS_ODOM_H
