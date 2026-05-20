#include "qualisys_ros2/QualisysOdom.h"
#include <Eigen/Geometry>

namespace qualisys
{

QualisysOdom::QualisysOdom() :
    Node("qualisys_odom")
{
}

bool QualisysOdom::init()
{
    double max_accel;
    this->declare_parameter("max_accel", 5.0);
    this->declare_parameter("qualisys_fps", 100.0);

    max_accel = this->get_parameter("max_accel").as_double();
    double qualisys_fps = this->get_parameter("qualisys_fps").as_double();

    if (qualisys_fps <= 0.0) {
        RCLCPP_FATAL(this->get_logger(), "qualisys_fps must be > 0");
        return false;
    }

    double dt = 1.0 / qualisys_fps;

    // Inicializa o KalmanFilter — igual ao ROS 1
    KalmanFilter::State_t proc_noise_diag;
    proc_noise_diag(0) = 0.5 * max_accel * dt * dt;
    proc_noise_diag(1) = 0.5 * max_accel * dt * dt;
    proc_noise_diag(2) = 0.5 * max_accel * dt * dt;
    proc_noise_diag(3) = max_accel * dt;
    proc_noise_diag(4) = max_accel * dt;
    proc_noise_diag(5) = max_accel * dt;
    proc_noise_diag = proc_noise_diag.array().square();

    KalmanFilter::Measurement_t meas_noise_diag;
    meas_noise_diag(0) = 1e-4;
    meas_noise_diag(1) = 1e-4;
    meas_noise_diag(2) = 1e-4;
    meas_noise_diag = meas_noise_diag.array().square();

    kf_.initialize(KalmanFilter::State_t::Zero(),
                   0.01 * KalmanFilter::ProcessCov_t::Identity(),
                   proc_noise_diag.asDiagonal(),
                   meas_noise_diag.asDiagonal());

    // Publisher e Subscriber — equivalente ao ROS 1
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    qualisys_sub_ = this->create_subscription<qualisys_ros2::msg::Subject>(
        "qualisys_subject",
        10,
        std::bind(&QualisysOdom::QualisysCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "QualisysOdom iniciado.");
    return true;
}

void QualisysOdom::QualisysCallback(
    const qualisys_ros2::msg::Subject::SharedPtr msg)
{
    static rclcpp::Time t_last_proc = msg->header.stamp;
    double dt = (rclcpp::Time(msg->header.stamp) - t_last_proc).seconds();
    t_last_proc = msg->header.stamp;

    // Kalman filter — igual ao ROS 1
    kf_.processUpdate(dt);
    const KalmanFilter::Measurement_t meas(
        msg->position.x,
        msg->position.y,
        msg->position.z
    );

    if (!msg->occluded) {
        static rclcpp::Time t_last_meas = msg->header.stamp;
        double meas_dt = (rclcpp::Time(msg->header.stamp) - t_last_meas).seconds();
        t_last_meas = msg->header.stamp;
        kf_.measurementUpdate(meas, meas_dt);
    }

    const KalmanFilter::State_t state = kf_.getState();
    const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();

    // Publica odometria — igual ao ROS 1
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header         = msg->header;
    odom_msg.child_frame_id = msg->name;

    odom_msg.pose.pose.position.x = state(0);
    odom_msg.pose.pose.position.y = state(1);
    odom_msg.pose.pose.position.z = state(2);

    odom_msg.twist.twist.linear.x = state(3);
    odom_msg.twist.twist.linear.y = state(4);
    odom_msg.twist.twist.linear.z = state(5);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            odom_msg.pose.covariance[6 * i + j]  = proc_noise(i, j);
            odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
        }
    }

    odom_msg.pose.pose.orientation = msg->orientation;

    // Velocidade angular — igual ao ROS 1
    static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d R(Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z));

    if (dt > 1e-6) {
        const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
        const Eigen::Matrix3d w_hat = R_dot * R.transpose();

        odom_msg.twist.twist.angular.x = w_hat(2, 1);
        odom_msg.twist.twist.angular.y = w_hat(0, 2);
        odom_msg.twist.twist.angular.z = w_hat(1, 0);
    }
    R_prev = R;

    odom_pub_->publish(odom_msg);
}

} // namespace qualisys
