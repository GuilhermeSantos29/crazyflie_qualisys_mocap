#include "qualisys_ros2/QualisysDriver.h"
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

namespace qualisys {

// Igual ao ROS 1
double QualisysDriver::deg2rad = M_PI / 180.0;

QualisysDriver::QualisysDriver() :
  Node("qualisys_driver"),
  publish_tf(false)
{
  // Parâmetros — equivalente ao nh.param() do ROS 1
  this->declare_parameter("server_address", string("141.23.110.143"));
  this->declare_parameter("server_base_port", 22222);
  this->declare_parameter("publish_tf", false);
}

bool QualisysDriver::init() {
  // Lê os parâmetros — equivalente ao nh.param() do ROS 1
  server_address = this->get_parameter("server_address").as_string();
  base_port      = this->get_parameter("server_base_port").as_int();
  publish_tf     = this->get_parameter("publish_tf").as_bool();

  // TF broadcaster
  tf_publisher = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Connecting to the server — igual ao ROS 1
  RCLCPP_INFO(this->get_logger(),
    "Connecting to the Qualisys Motion Tracking system specified at: %s:%d",
    server_address.c_str(), base_port);

  if (!port_protocol.Connect((char*)server_address.data(), base_port, 0, 1, 7)) {
    RCLCPP_FATAL(this->get_logger(),
      "Could not find the Qualisys Motion Tracking system at: %s:%d",
      server_address.c_str(), base_port);
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
    "Connected to %s:%d", server_address.c_str(), base_port);

  // Get 6DOF settings — igual ao ROS 1
  port_protocol.Read6DOFSettings();

  return true;
}

void QualisysDriver::disconnect() {
  RCLCPP_INFO(this->get_logger(),
    "Disconnected with the server %s:%d",
    server_address.c_str(), base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
}

void QualisysDriver::checkPublishers(const int& body_count) {
  map<string, bool> subject_indicator;

  for (auto it = subject_publishers.begin();
      it != subject_publishers.end(); ++it)
    subject_indicator[it->first] = false;

  // Check publishers for each body — igual ao ROS 1
  for (int i = 0; i < body_count; ++i) {
    string name(port_protocol.Get6DOFBodyName(i));

    // Cria publisher se não existir
    // equivalente ao nh.advertise<T>() do ROS 1
    if (subject_publishers.find(name) == subject_publishers.end()) {
      subject_publishers[name] =
        this->create_publisher<qualisys_ros2::msg::Subject>(name, 10);
    }

    subject_indicator[name] = true;
  }

  for (auto it = subject_indicator.begin();
      it != subject_indicator.end(); ++it) {
    if (it->second == false)
      subject_publishers.erase(it->first);
  }
}

void QualisysDriver::handlePacketData(CRTPacket* prt_packet) {

  // Número de rigid bodies — igual ao ROS 1
  int body_count = prt_packet->Get6DOFEulerBodyCount();

  checkPublishers(body_count);

  for (int i = 0; i < body_count; ++i) {
    float x, y, z, roll, pitch, yaw;
    prt_packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

    if (isnan(x) || isnan(y) || isnan(z) ||
        isnan(roll) || isnan(pitch) || isnan(yaw)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
        *this->get_clock(), 3000,
        "Rigid-body %d/%d not detected", i + 1, body_count);
      continue;
    }

    string subject_name(port_protocol.Get6DOFBodyName(i));

    // Correcção do flip de 180 graus — igual ao ROS 1
    if (roll > 90)
      roll -= 180;
    else if (roll < -90)
      roll += 180;

    // Converte Euler para quaternião — equivalente ao tf::createQuaternionFromRPY
    tf2::Quaternion q;
    q.setRPY(roll * deg2rad, pitch * deg2rad, yaw * deg2rad);

    // Envia TF — equivalente ao tf_publisher.sendTransform() do ROS 1
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = this->get_clock()->now();
    transform_stamped.header.frame_id = "qualisys";
    transform_stamped.child_frame_id  = subject_name;
    transform_stamped.transform.translation.x = x / 1000.0;
    transform_stamped.transform.translation.y = y / 1000.0;
    transform_stamped.transform.translation.z = z / 1000.0;
    transform_stamped.transform.rotation      = tf2::toMsg(q);

    if (publish_tf)
      tf_publisher->sendTransform(transform_stamped);

    // Publica Subject msg — equivalente ao subject_publishers[name].publish() do ROS 1
    qualisys_ros2::msg::Subject subject_msg;
    subject_msg.header        = transform_stamped.header;
    subject_msg.name          = subject_name;
    subject_msg.position.x    = transform_stamped.transform.translation.x;
    subject_msg.position.y    = transform_stamped.transform.translation.y;
    subject_msg.position.z    = transform_stamped.transform.translation.z;
    subject_msg.orientation   = transform_stamped.transform.rotation;
    subject_msg.occluded      = false;

    subject_publishers[subject_name]->publish(subject_msg);
  }
}

void QualisysDriver::run() {

  CRTPacket* prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

  if (port_protocol.ReceiveRTPacket(e_type, true)) {

    switch (e_type) {
      // Erro — igual ao ROS 1
      case CRTPacket::PacketError:
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
          *this->get_clock(), 1000,
          "Error when streaming frames: %s",
          port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Sem mais dados — igual ao ROS 1
      case CRTPacket::PacketNoMoreData:
        RCLCPP_WARN_THROTTLE(this->get_logger(),
          *this->get_clock(), 1000,
          "No more data");
        break;

      // Dados recebidos — igual ao ROS 1
      case CRTPacket::PacketData:
        handlePacketData(prt_packet);
        break;

      default:
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
          *this->get_clock(), 1000,
          "Unknown CRTPacket case");
    }
  }
}

} // namespace qualisys
