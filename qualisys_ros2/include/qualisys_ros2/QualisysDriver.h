#ifndef QUALISYS_DRIVER_H
#define QUALISYS_DRIVER_H

#include <sstream>
#include <cmath>
#include <string>
#include <map>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Mensagens do package
#include <qualisys_ros2/msg/subject.hpp>

// Protocolo QTM (igual ao ROS 1 — não muda)
#include "RTProtocol.h"

namespace qualisys {

class QualisysDriver : public rclcpp::Node {

  public:
    /*
     * @brief Constructor
     */
    QualisysDriver();

    /*
     * @brief Destructor
     */
    ~QualisysDriver() {
      disconnect();
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

    /*
     * @brief run Start acquiring data from the server
     */
    void run();

    /*
     * @brief disconnect Disconnect to the server
     */
    void disconnect();

  private:
    // Disable the copy constructor and assign operator
    QualisysDriver(const QualisysDriver& );
    QualisysDriver& operator=(const QualisysDriver& );

    // Initialize publishers
    void checkPublishers(const int& body_count);

    // Handle data packet
    void handlePacketData(CRTPacket* prt_packet);

    // Unit converter
    static double deg2rad;

    // Address and port of the server
    std::string server_address;
    int base_port;

    // Protocol to connect to the server (igual ao ROS 1 — não muda)
    CRTProtocol port_protocol;

    // Publishers
    std::map<std::string, rclcpp::Publisher<qualisys_ros2::msg::Subject>::SharedPtr> subject_publishers;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher;

    // If publish tf msgs
    bool publish_tf;
};

} // namespace qualisys

#endif
