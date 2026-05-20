"""
drone_controller_node.py

Nó ROS 2 que gere a missão de voo do Crazyflie 2.1 Brushless.
Subscreve a pose do drone via MoCap e controla o voo através
dos serviços de alto nível do crazyswarm2 (takeoff, go_to, land).

A trajectória e os parâmetros de voo são configurados em
config/params_controller.yaml.
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm, UpdateParams
from crazyflie_interfaces.msg import VelocityWorld
from std_msgs.msg import Header

from mocap_bridge.flight_handler import FlightHandler


class DroneControllerNode(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # Lê os parâmetros do ficheiro params_controller.yaml
        self.declare_parameter('robot_name',        'cf231')
        self.declare_parameter('target_height',      1.0)
        self.declare_parameter('takeoff_duration',   2.5)
        self.declare_parameter('hover_duration',     3.0)
        self.declare_parameter('land_duration',      2.5)
        self.declare_parameter('mode',              'position')
        self.declare_parameter('trajectory',        'square')
        self.declare_parameter('side_length',        1.0)
        self.declare_parameter('velocity',           0.5)
        self.declare_parameter('waypoint_duration',  3.0)
        self.declare_parameter('circle_points',      16)

        robot_name             = self.get_parameter('robot_name').get_parameter_value().string_value
        target_height          = self.get_parameter('target_height').get_parameter_value().double_value
        self.takeoff_duration  = self.get_parameter('takeoff_duration').get_parameter_value().double_value
        self.hover_duration    = self.get_parameter('hover_duration').get_parameter_value().double_value
        self.land_duration     = self.get_parameter('land_duration').get_parameter_value().double_value
        self.mode              = self.get_parameter('mode').get_parameter_value().string_value
        trajectory             = self.get_parameter('trajectory').get_parameter_value().string_value
        side_length            = self.get_parameter('side_length').get_parameter_value().double_value
        velocity               = self.get_parameter('velocity').get_parameter_value().double_value
        self.waypoint_duration = self.get_parameter('waypoint_duration').get_parameter_value().double_value
        circle_points          = self.get_parameter('circle_points').get_parameter_value().integer_value

        # Flags de controlo da missão
        self.mission_complete = False  # impede que a missão se repita após aterrar
        self.waypoint_sent    = False  # garante que o go_to só é enviado uma vez por waypoint
        self.rotation_done    = False  # controla a rotação antes de avançar (só no quadrado)

        # O flight_handler gere a lógica de voo independentemente do ROS
        self.handler = FlightHandler(target_height=target_height, side_length=side_length, mode=self.mode, velocity=velocity, trajectory=trajectory, circle_points=circle_points)

        # Subscrição à pose do drone publicada pelo mocap_bridge
        self.sub_pose = self.create_subscription(
            PoseStamped,
            f'/{robot_name}/pose',
            self.pose_callback,
            10
        )

        # Clientes dos serviços do crazyswarm2
        self.cli_arm           = self.create_client(Arm,          f'/{robot_name}/arm')
        self.cli_takeoff       = self.create_client(Takeoff,      f'/{robot_name}/takeoff')
        self.cli_land          = self.create_client(Land,         f'/{robot_name}/land')
        self.cli_goto          = self.create_client(GoTo,         f'/{robot_name}/go_to')
        self.cli_update_params = self.create_client(UpdateParams, f'/{robot_name}/update_params')

        # Publisher de velocidades para o modo velocity
        self.pub_vel = self.create_publisher(
            VelocityWorld,
            f'/{robot_name}/cmd_vel_legacy',
            10
        )

        # Timer principal — verifica o estado do drone a cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'DroneControllerNode iniciado:\n'
            f'  robot_name      : {robot_name}\n'
            f'  mode            : {self.mode}\n'
            f'  trajectory      : {trajectory}\n'
            f'  target_height   : {target_height} m\n'
            f'  side_length     : {side_length} m\n'
            f'  velocity        : {velocity} m/s\n'
            f'  takeoff_duration: {self.takeoff_duration} s\n'
            f'  hover_duration  : {self.hover_duration} s\n'
            f'  land_duration   : {self.land_duration} s'
        )

    def pose_callback(self, msg: PoseStamped):
        # Passa a pose actual ao flight_handler para que ele saiba onde o drone está
        self.handler.update_pose(msg.pose)
        self.get_logger().debug(
            f'[drone_controller] x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} '
            f'z={msg.pose.position.z:.3f} '
            f'estado={self.handler.get_state()}'
        )

    def timer_callback(self):
        # Se a missão já terminou não faz nada
        if self.mission_complete:
            return

        state = self.handler.get_state()

        if state == FlightHandler.LANDED:
            # Só inicia o voo quando o MoCap já está a fornecer a pose
            if self.handler.is_ready_to_fly():
                self.do_takeoff()

        elif state == FlightHandler.FLYING:
            if self.handler.trajectory == FlightHandler.TRAJECTORY_HOVER:
                return
            elif self.mode == FlightHandler.MODE_POSITION:
                self.fly_position()
            else:
                self.fly_velocity()

    def reset_estimator(self):
        # Reinicia o filtro de Kalman do drone para garantir que a estimativa
        # de posição converge para os dados do MoCap antes do takeoff
        if not self.cli_update_params.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[drone_controller] update_params não disponível.')
            return

        req = UpdateParams.Request()
        req.params = ['kalman.resetEstimation']
        self.cli_update_params.call_async(req)
        self.get_logger().info('[drone_controller] Kalman reiniciado.')
        time.sleep(2.0)

    def do_arm(self):
        # O Crazyflie 2.1 Brushless requer arm explícito antes de qualquer voo
        if not self.cli_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[drone_controller] Serviço arm não disponível.')
            return

        req = Arm.Request()
        req.arm = True
        self.cli_arm.call_async(req)
        self.get_logger().info('[drone_controller] Motores armados.')

    def do_takeoff(self):
        if not self.cli_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[drone_controller] Serviço takeoff não disponível.')
            return

        # Reinicia o Kalman e arma os motores antes de subir
        self.reset_estimator()
        self.do_arm()
        time.sleep(1.0)

        req = Takeoff.Request()
        req.height   = self.handler.target_height
        req.duration = rclpy.duration.Duration(seconds=self.takeoff_duration).to_msg()

        self.handler.set_state(FlightHandler.TAKEOFF)
        self.get_logger().info(
            f'[drone_controller] Takeoff para {self.handler.target_height} m.'
        )

        future = self.cli_takeoff.call_async(req)
        future.add_done_callback(self.takeoff_done_callback)

    def takeoff_done_callback(self, future):
        self.get_logger().info('[drone_controller] Takeoff concluído — a pairar.')
        self.handler.set_state(FlightHandler.HOVER)
        # Aguarda hover_duration segundos antes de iniciar a trajectória
        self._start_traj_timer = self.create_timer(self.hover_duration, self.start_trajectory)

    def start_trajectory(self):
        self._start_traj_timer.cancel()

        self.handler.set_state(FlightHandler.FLYING)
        self.handler.reset_trajectory()
        self.waypoint_sent = False
        self.rotation_done = False
        self.get_logger().info(
            f'[drone_controller] Trajectória iniciada: {self.handler.trajectory}'
        )

        # No hover agenda o land após hover_duration segundos
        if self.handler.trajectory == FlightHandler.TRAJECTORY_HOVER:
            self._land_timer = self.create_timer(self.hover_duration, self.do_land)

    def _direction_to_waypoint(self, tx: float, ty: float) -> float:
        # Calcula o ângulo (yaw) entre a posição actual e o waypoint destino
        if self.handler.current_pose is None:
            return 0.0
        cx = self.handler.current_pose.position.x
        cy = self.handler.current_pose.position.y
        dx, dy = tx - cx, ty - cy
        if math.sqrt(dx**2 + dy**2) < 0.05:
            return 0.0
        return math.atan2(dy, dx)

    def _get_yaw(self, x: float, y: float) -> float:
        # Só roda no quadrado — nas outras trajectórias mantém yaw=0
        if self.handler.trajectory == FlightHandler.TRAJECTORY_SQUARE:
            return self._direction_to_waypoint(x, y)
        return 0.0

    def fly_position(self):
        if self.handler.is_trajectory_complete():
            self.do_land()
            return

        if self.handler.reached_waypoint():
            self.get_logger().info(
                f'[drone_controller] Waypoint {self.handler.current_waypoint + 1} atingido.'
            )
            self.handler.advance_waypoint()
            self.waypoint_sent = False
            self.rotation_done = False
            if self.handler.is_trajectory_complete():
                self.do_land()
                return

        # Só envia o go_to uma vez por waypoint para evitar comandos repetidos
        if self.waypoint_sent:
            return

        waypoint = self.handler.get_next_waypoint()
        if waypoint is None or self.handler.current_pose is None:
            return

        if not self.cli_goto.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[drone_controller] Serviço go_to não disponível.')
            return

        x, y, z = waypoint
        yaw = self._get_yaw(x, y)

        # No quadrado: roda primeiro no sítio, depois avança
        if self.handler.trajectory == FlightHandler.TRAJECTORY_SQUARE:
            cx = self.handler.current_pose.position.x
            cy = self.handler.current_pose.position.y
            cz = self.handler.current_pose.position.z

            req_rot = GoTo.Request()
            req_rot.goal.x   = cx
            req_rot.goal.y   = cy
            req_rot.goal.z   = cz
            req_rot.yaw      = yaw
            req_rot.duration = rclpy.duration.Duration(seconds=1.5).to_msg()
            req_rot.relative = False
            self.cli_goto.call_async(req_rot)
            self.get_logger().info(
                f'[drone_controller] A rodar {math.degrees(yaw):.1f}°'
            )
            self.waypoint_sent = True
            self._rotation_timer = self.create_timer(1.5, self._advance_to_waypoint)

        # Nas outras trajectórias: avança directamente
        else:
            req = GoTo.Request()
            req.goal.x   = x
            req.goal.y   = y
            req.goal.z   = z
            req.yaw      = 0.0
            req.duration = rclpy.duration.Duration(seconds=self.waypoint_duration).to_msg()
            req.relative = False
            self.cli_goto.call_async(req)
            self.waypoint_sent = True
            self.get_logger().info(
                f'[drone_controller] Waypoint x={x:.2f} y={y:.2f} z={z:.2f}'
            )

    def _advance_to_waypoint(self):
        # Chamado após a rotação no quadrado — envia o go_to para a posição destino
        self._rotation_timer.cancel()

        waypoint = self.handler.get_next_waypoint()
        if waypoint is None:
            return

        if not self.cli_goto.wait_for_service(timeout_sec=1.0):
            return

        x, y, z = waypoint
        yaw = self._get_yaw(x, y)

        req = GoTo.Request()
        req.goal.x   = x
        req.goal.y   = y
        req.goal.z   = z
        req.yaw      = yaw
        req.duration = rclpy.duration.Duration(seconds=self.waypoint_duration).to_msg()
        req.relative = False
        self.cli_goto.call_async(req)
        self.get_logger().info(
            f'[drone_controller] A avançar x={x:.2f} y={y:.2f} z={z:.2f} '
            f'yaw={math.degrees(yaw):.1f}°'
        )

    def fly_velocity(self):
        if self.handler.is_trajectory_complete():
            self.do_land()
            return

        if self.handler.reached_waypoint():
            self.handler.advance_waypoint()
            if self.handler.is_trajectory_complete():
                self.do_land()
                return

        vx, vy, vz = self.handler.get_velocity_command()

        msg = VelocityWorld()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world/drone_controller'
        msg.vel.x    = vx
        msg.vel.y    = vy
        msg.vel.z    = vz
        msg.yaw_rate = 0.0
        self.pub_vel.publish(msg)

    def do_land(self):
        if self.mission_complete:
            return

        if hasattr(self, '_land_timer'):
            self._land_timer.cancel()

        if not self.cli_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('[drone_controller] Serviço land não disponível.')
            return

        req = Land.Request()
        req.height   = 0.04
        req.duration = rclpy.duration.Duration(seconds=self.land_duration).to_msg()

        self.handler.set_state(FlightHandler.LANDING)
        self.get_logger().info('[drone_controller] A aterrar.')

        self.cli_land.call_async(req)
        # Usa timer em vez de callback para não bloquear o executor ROS 2
        self._mission_timer = self.create_timer(self.land_duration + 1.0, self.mission_done)

    def mission_done(self):
        self._mission_timer.cancel()
        if self.mission_complete:
            return
        self.handler.set_state(FlightHandler.LANDED)
        self.mission_complete = True
        self.get_logger().info('[drone_controller] Missão terminada.')


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
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
