"""
node.py
-------
Nó ROS 2 responsável por:
  - Subscrever o tópico /<robot_name> publicado pelo qualisys_ros2
  - Validar a pose com o pose_handler
  - Publicar em /poses (NamedPoseArray) para o crazyflie_server
  - Publicar em /<robot_name>/pose (PoseStamped) para o drone_controller

Toda a lógica de validação está em pose_handler.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose
from rclpy.duration import Duration

from qualisys_ros2.msg import Subject
from mocap_bridge.pose_handler import PoseHandler


class MocapBridgeNode(Node):

    def __init__(self):
        super().__init__('mocap_bridge')

        # --- Parâmetros ---
        self.declare_parameter('robot_name',   'cf231')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('frame_id',     'world')
        self.declare_parameter('max_position',  10.0)

        self.robot_name  = self.get_parameter('robot_name').get_parameter_value().string_value
        output_topic     = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id    = self.get_parameter('frame_id').get_parameter_value().string_value
        max_position     = self.get_parameter('max_position').get_parameter_value().double_value

        # Tópico de saída por defeito: /<robot_name>/pose
        if not output_topic:
            output_topic = f'/{self.robot_name}/pose'

        # --- Handler de poses (lógica independente de ROS) ---
        self.handler = PoseHandler(
            robot_name=self.robot_name,
            max_position=max_position
        )

        # --- QoS para o tópico do qualisys_ros2 ---
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- QoS para o /poses (igual ao motion_capture_tracking) ---
        qos_mocap = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            deadline=Duration(nanoseconds=10000000)  #50000000
        )

        # --- Subscrição ao tópico /<robot_name> do qualisys_ros2 ---
        self.sub_subject = self.create_subscription(
            Subject,
            f'/{self.robot_name}',
            self.subject_callback,
            qos_sensor
        )

        # --- Publisher /cf231/pose (PoseStamped) para o drone_controller ---
        self.pub_pose = self.create_publisher(
            PoseStamped,
            output_topic,
            10
        )

        # --- Publisher /poses (NamedPoseArray) para o crazyflie_server ---
        self.pub_named_poses = self.create_publisher(
            NamedPoseArray,
            '/poses',
            qos_mocap
        )

        self.get_logger().info(
            f'MocapBridgeNode iniciado:\n'
            f'  robot_name   : {self.robot_name}\n'
            f'  input_topic  : /{self.robot_name}\n'
            f'  output_topic : {output_topic}\n'
            f'  frame_id     : {self.frame_id}'
        )

    def subject_callback(self, msg: Subject):
        """
        Callback chamado cada vez que chega um Subject do qualisys_ros2.
        Extrai a pose, valida e publica em /poses e /<robot_name>/pose.
        """
        # Se o rigid body está ocluído (não visível pelas câmeras)
        if msg.occluded:
            self.get_logger().warn(
                '[mocap_bridge] Rigid body ocluído — pose ignorada.',
                throttle_duration_sec=5.0
            )
            return

        # Constrói uma Pose a partir do Subject
        raw_pose = Pose()
        raw_pose.position    = msg.position
        raw_pose.orientation = msg.orientation

        # Delega a validação ao PoseHandler
        processed_pose = self.handler.process(raw_pose)

        if processed_pose is None:
            self.get_logger().warn(
                '[mocap_bridge] Pose inválida recebida — ignorada.',
                throttle_duration_sec=5.0
            )
            return

        timestamp = self.get_clock().now().to_msg()

        # --- Publica como PoseStamped em /<robot_name>/pose ---
        msg_out = PoseStamped()
        msg_out.header = Header()
        msg_out.header.stamp = timestamp
        msg_out.header.frame_id = f'{self.frame_id}/mocap_bridge'
        msg_out.pose = processed_pose
        self.pub_pose.publish(msg_out)

        # --- Publica como NamedPoseArray em /poses para o crazyflie_server ---
        named_pose = NamedPose()
        named_pose.name = self.robot_name
        named_pose.pose = processed_pose

        named_pose_array = NamedPoseArray()
        named_pose_array.header = Header()
        named_pose_array.header.stamp = timestamp
        named_pose_array.header.frame_id = 'mocap'
        named_pose_array.poses = [named_pose]
        self.pub_named_poses.publish(named_pose_array)

        self.get_logger().debug(
            f'[mocap_bridge] Pose publicada: '
            f'x={processed_pose.position.x:.3f}, '
            f'y={processed_pose.position.y:.3f}, '
            f'z={processed_pose.position.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
