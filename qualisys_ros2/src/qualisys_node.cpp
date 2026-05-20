#include <rclcpp/rclcpp.hpp>
#include "qualisys_ros2/QualisysDriver.h"

int main(int argc, char *argv[]) {

  // Equivalente ao ros::init() do ROS 1
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<qualisys::QualisysDriver>();

  if (!driver->init()) {
    RCLCPP_FATAL(driver->get_logger(),
      "Initialization of the qualisys driver failed!");
    return -1;
  }

  // Equivalente ao while(ros::ok()) + ros::spinOnce() do ROS 1
  while (rclcpp::ok()) {
    driver->run();
    rclcpp::spin_some(driver);
  }

  RCLCPP_INFO(driver->get_logger(), "Shutting down");
  driver->disconnect();

  rclcpp::shutdown();
  return 0;
}
