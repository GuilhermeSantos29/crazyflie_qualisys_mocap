#ifndef QUALISYS_CALIB_H
#define QUALISYS_CALIB_H

#include <map>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Mensagens do package
#include <qualisys_ros2/msg/subject.hpp>

namespace qualisys {

class QualisysCalib : public rclcpp::Node {
public:
    QualisysCalib();
    ~QualisysCalib() {}

    bool init();

private:
    std::string calib_marker_pos_file;
    std::string zero_pose_dir;
    std::string model_name;

    std::map<std::string, Eigen::Vector3d> marker_pos_map;
    std::vector<Eigen::Vector3d> calib_ref_points;
    std::vector<Eigen::Vector3d> calib_actual_points;
    Eigen::Affine3d calib_transform;

    bool enable_calibration;
    bool calib_stand_ready;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr zero_pose_pub_;
    rclcpp::Subscription<qualisys_ros2::msg::Subject>::SharedPtr calib_sub_;
    rclcpp::Subscription<qualisys_ros2::msg::Subject>::SharedPtr subject_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_calib_srv_;

    // Disable copy constructor and assign operator
    QualisysCalib(const QualisysCalib&);
    QualisysCalib& operator=(const QualisysCalib&);

    bool loadZeroPoseFromFile(const std::string &filename, Eigen::Affine3d &zero_pose);
    bool saveZeroPoseToFile(const Eigen::Affine3d &zero_pose, const std::string &filename);
    bool loadCalibMarkerPos(const std::string &filename, std::map<std::string, Eigen::Vector3d> &marker_pos_map);
    bool getTransform(const std::vector<Eigen::Vector3d> &reference_points,
                      const std::vector<Eigen::Vector3d> &actual_points,
                      Eigen::Affine3d &transform);

    void calibStandCallback(const qualisys_ros2::msg::Subject::SharedPtr msg);
    void subjectCallback(const qualisys_ros2::msg::Subject::SharedPtr msg);
    void toggleCalibCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);
};

} // namespace qualisys

#endif
