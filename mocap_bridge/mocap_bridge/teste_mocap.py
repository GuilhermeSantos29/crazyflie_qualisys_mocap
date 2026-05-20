"""
teste_mocap.py
--------------
Nó de teste que simula o motion_capture_tracking.
Publica poses falsas no tópico /poses para testar o mocap_bridge
sem precisar de estar ligado à rede da arena.
Simula o drone a levantar voo, fazer um quadrado e pousar (uma vez).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math


class TesteMocapNode(Node):

    def __init__(self):
        super().__init__('teste_mocap')

        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('max_height', 1.0)
        self.declare_parameter('radius', 0.5)

        frequency       = self.get_parameter('frequency').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.radius     = self.get_parameter('radius').get_parameter_value().double_value

        self.pub   = self.create_publisher(PoseArray, '/poses', 10)
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)#timer para 50 vezes por segundo

        self.t  = 0.0
        self.dt = 1.0 / frequency #intervalo de tempo entre cada publicação (0.02s)

        self.get_logger().info(
            f'TesteMocapNode iniciado — a publicar em /poses a {frequency} Hz'
        )

    def timer_callback(self):
        self.t += self.dt

        pose = Pose()

        # Fase 1 (0s - 3s): subir
        if self.t < 3.0:
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = (self.t / 3.0) * self.max_height

        # Fase 2 (3s - 9s): quadrado
        elif self.t < 9.0:
            t_quad = (self.t - 3.0)  # tempo dentro da fase (0 a 6 segundos)
            lado = self.radius        # tamanho do lado do quadrado

            if t_quad < 1.5:         # lado 1: frente
                pose.position.x = (t_quad / 1.5) * lado
                pose.position.y = 0.0
            elif t_quad < 3.0:       # lado 2: direita
                pose.position.x = lado
                pose.position.y = ((t_quad - 1.5) / 1.5) * lado
            elif t_quad < 4.5:       # lado 3: trás
                pose.position.x = lado - ((t_quad - 3.0) / 1.5) * lado
                pose.position.y = lado
            else:                    # lado 4: esquerda
                pose.position.x = 0.0
                pose.position.y = lado - ((t_quad - 4.5) / 1.5) * lado

            pose.position.z = self.max_height

        # Fase 3 (9s - 12s): pousar
        elif self.t < 12.0:
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = self.max_height * (1.0 - (self.t - 9.0) / 3.0)

        # Simulação concluída
        else:
            self.get_logger().info('Simulação concluída!')
            self.timer.cancel()
            return

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world/teste_mocap'
        msg.poses = [pose]

        self.pub.publish(msg)
        
        self.get_logger().debug(
            f'[teste_mocap] Pose publicada: '
            f'x={pose.position.x:.3f}, '
            f'y={pose.position.y:.3f}, '
            f'z={pose.position.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TesteMocapNode()
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
