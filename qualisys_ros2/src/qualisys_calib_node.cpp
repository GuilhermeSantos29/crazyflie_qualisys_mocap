#include <rclcpp/rclcpp.hpp>
#include "qualisys_ros2/QualisysCalib.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<qualisys::QualisysCalib>();

    if (!node->init()) {
        RCLCPP_FATAL(node->get_logger(),
            "Calibration initialization failed!");
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
