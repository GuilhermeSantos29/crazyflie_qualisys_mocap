#include "qualisys_ros2/QualisysCalib.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace qualisys
{

QualisysCalib::QualisysCalib() :
    Node("qualisys_calib"),
    enable_calibration(false),
    calib_stand_ready(false)
{
}

bool QualisysCalib::init()
{
    this->declare_parameter("calib_marker_pos_file", string("QuadrotorCalib.yaml"));
    this->declare_parameter("zero_pose_dir", string("calib"));

    calib_marker_pos_file = this->get_parameter("calib_marker_pos_file").as_string();
    zero_pose_dir         = this->get_parameter("zero_pose_dir").as_string();

    if (!loadCalibMarkerPos(calib_marker_pos_file, marker_pos_map)) {
        RCLCPP_ERROR(this->get_logger(),
            "Error loading calib marker positions from file: %s",
            calib_marker_pos_file.c_str());
        return false;
    }

    calib_ref_points.resize(marker_pos_map.size());
    calib_actual_points.resize(marker_pos_map.size());

    // Publishers, Subscribers e Serviços — equivalente ao ROS 1
    zero_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("zero_pose", 10);

    calib_sub_ = this->create_subscription<qualisys_ros2::msg::Subject>(
        "qualisys_calib", 10,
        std::bind(&QualisysCalib::calibStandCallback, this, std::placeholders::_1)
    );

    subject_sub_ = this->create_subscription<qualisys_ros2::msg::Subject>(
        "qualisys_subject", 10,
        std::bind(&QualisysCalib::subjectCallback, this, std::placeholders::_1)
    );

    // Serviço — equivalente ao nh.advertiseService() do ROS 1
    toggle_calib_srv_ = this->create_service<std_srvs::srv::Empty>(
        "toggle_calibration",
        std::bind(&QualisysCalib::toggleCalibCallback, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "QualisysCalib iniciado.");
    return true;
}

bool QualisysCalib::loadZeroPoseFromFile(
    const std::string &filename, Eigen::Affine3d &zero_pose)
{
    zero_pose = Eigen::Affine3d::Identity();

    std::ifstream fin(filename.c_str());
    if (!fin.is_open())
        return false;

    try {
        YAML::Node doc = YAML::Load(fin);
        Eigen::Vector3d v;
        Eigen::Quaterniond q;
        v(0) = doc["translation"]["x"].as<double>();
        v(1) = doc["translation"]["y"].as<double>();
        v(2) = doc["translation"]["z"].as<double>();
        q.x() = doc["rotation"]["x"].as<double>();
        q.y() = doc["rotation"]["y"].as<double>();
        q.z() = doc["rotation"]["z"].as<double>();
        q.w() = doc["rotation"]["w"].as<double>();
        zero_pose.translate(v);
        zero_pose.rotate(q);
    } catch (...) {
        fin.close();
        return false;
    }

    fin.close();
    return true;
}

bool QualisysCalib::saveZeroPoseToFile(
    const Eigen::Affine3d &zero_pose, const std::string &filename)
{
    YAML::Emitter out;
    Eigen::Vector3d v(zero_pose.translation());
    Eigen::Quaterniond q(zero_pose.rotation());

    out << YAML::BeginMap;
    out << YAML::Key << "translation";
    out << YAML::Value << YAML::BeginMap
        << YAML::Key << "x" << YAML::Value << v.x()
        << YAML::Key << "y" << YAML::Value << v.y()
        << YAML::Key << "z" << YAML::Value << v.z()
        << YAML::EndMap;
    out << YAML::Key << "rotation";
    out << YAML::Value << YAML::BeginMap
        << YAML::Key << "x" << YAML::Value << q.x()
        << YAML::Key << "y" << YAML::Value << q.y()
        << YAML::Key << "z" << YAML::Value << q.z()
        << YAML::Key << "w" << YAML::Value << q.w()
        << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream fout(filename.c_str(),
        std::ios_base::out | std::ios_base::trunc);
    if (!fout.is_open())
        return false;

    fout << out.c_str() << std::endl;
    fout.close();
    return true;
}

bool QualisysCalib::getTransform(
    const std::vector<Eigen::Vector3d> &reference_points,
    const std::vector<Eigen::Vector3d> &actual_points,
    Eigen::Affine3d &transform)
{
    transform = Eigen::Affine3d::Identity();

    if (reference_points.size() != actual_points.size())
        return false;

    const size_t num_points = reference_points.size();
    Eigen::Vector3d reference_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_mean    = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < num_points; i++) {
        reference_mean += reference_points[i];
        actual_mean    += actual_points[i];
    }
    reference_mean /= num_points;
    actual_mean    /= num_points;

    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < num_points; i++) {
        C += (actual_points[i] - actual_mean) *
             (reference_points[i] - reference_mean).transpose();
    }
    C /= num_points;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d S       = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0)
        S(2, 2) = -1;

    const Eigen::Matrix3d rotation    = U * S * V.transpose();
    const Eigen::Vector3d translation = actual_mean - rotation * reference_mean;

    transform.translate(translation);
    transform.rotate(rotation);
    return true;
}

bool QualisysCalib::loadCalibMarkerPos(
    const std::string &filename,
    std::map<std::string, Eigen::Vector3d> &marker_pos_map)
{
    std::ifstream fin(filename.c_str());
    if (!fin.is_open())
        return false;

    try {
        YAML::Node doc = YAML::Load(fin);
        const YAML::Node &markers = doc["markers"];
        for (unsigned int i = 0; i < markers.size(); i++) {
            std::string name = markers[i]["name"].as<std::string>();
            const YAML::Node &pos = markers[i]["position"];
            Eigen::Vector3d position;
            position.x() = pos[0].as<double>();
            position.y() = pos[1].as<double>();
            position.z() = pos[2].as<double>();
            marker_pos_map[name] = position;
        }
    } catch (...) {
        fin.close();
        marker_pos_map.clear();
        return false;
    }

    fin.close();
    return true;
}

void QualisysCalib::calibStandCallback(
    const qualisys_ros2::msg::Subject::SharedPtr msg)
{
    calib_ref_points.clear();
    calib_actual_points.clear();

    for (size_t i = 0; i < msg->markers.size(); i++) {
        if (!msg->markers[i].occluded) {
            auto it = marker_pos_map.find(msg->markers[i].name);
            if (it != marker_pos_map.end()) {
                calib_ref_points.push_back(it->second);
                calib_actual_points.push_back(Eigen::Vector3d(
                    msg->markers[i].position.x,
                    msg->markers[i].position.y,
                    msg->markers[i].position.z));
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Marker %s not in calib file, skipping",
                    msg->markers[i].name.c_str());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Marker %s occluded, skipping",
                msg->markers[i].name.c_str());
        }
    }

    if (!getTransform(calib_ref_points, calib_actual_points, calib_transform)) {
        RCLCPP_WARN(this->get_logger(), "QualisysCalib::getTransform failed");
        calib_stand_ready = false;
    } else {
        calib_stand_ready = true;
    }
}

void QualisysCalib::subjectCallback(
    const qualisys_ros2::msg::Subject::SharedPtr msg)
{
    static bool calibrating = false;
    static double t_x, t_y, t_z;
    static double q_x, q_y, q_z, q_w;
    static unsigned int count;

    if (enable_calibration && !calibrating) {
        t_x = t_y = t_z = 0;
        q_x = q_y = q_z = q_w = 0;
        count = 0;
        calibrating = true;
    } else if (!enable_calibration && calibrating) {
        calibrating = false;

        Eigen::Affine3d zero_pose;
        zero_pose.setIdentity();
        Eigen::Quaterniond q(q_w / count, q_x / count, q_y / count, q_z / count);
        q.normalize();
        Eigen::Vector3d t(t_x / count, t_y / count, t_z / count);
        zero_pose.translate(t);
        zero_pose.rotate(q);

        model_name = msg->name;
        string model_zero_pose_file = zero_pose_dir + "/" + model_name + ".yaml";
        saveZeroPoseToFile(zero_pose, model_zero_pose_file);
    }

    if (calibrating) {
        Eigen::Affine3d current_pose;
        Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
        Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                             msg->orientation.y, msg->orientation.z);
        model_name = msg->name;
        current_pose.setIdentity();
        current_pose.translate(t);
        current_pose.rotate(q);

        Eigen::Affine3d zero_pose = calib_transform.inverse() * current_pose;
        t = zero_pose.translation();
        q = Eigen::Quaterniond(zero_pose.rotation());

        t_x += t(0); t_y += t(1); t_z += t(2);
        q_x += q.x(); q_y += q.y(); q_z += q.z(); q_w += q.w();
        count++;

        auto zero_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        zero_pose_msg->header.stamp    = msg->header.stamp;
        zero_pose_msg->header.frame_id = "qualisys";
        zero_pose_msg->pose.position.x = t_x / count;
        zero_pose_msg->pose.position.y = t_y / count;
        zero_pose_msg->pose.position.z = t_z / count;

        q = Eigen::Quaterniond(q_w / count, q_x / count, q_y / count, q_z / count);
        q.normalize();
        zero_pose_msg->pose.orientation.x = q.x();
        zero_pose_msg->pose.orientation.y = q.y();
        zero_pose_msg->pose.orientation.z = q.z();
        zero_pose_msg->pose.orientation.w = q.w();

        zero_pose_pub_->publish(*zero_pose_msg);
    }
}

void QualisysCalib::toggleCalibCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
{
    if (!calib_stand_ready) {
        RCLCPP_ERROR(this->get_logger(),
            "Do not have calib stand pose, cannot calibrate");
        enable_calibration = false;
        return;
    }
    enable_calibration = !enable_calibration;
}

} // namespace qualisys
