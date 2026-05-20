"""
cflib_controller_node.py
------------------------
Nó ROS 2 que:
  - Subscreve /cf231/pose (posição do MoCap via mocap_bridge)
  - Envia a posição ao drone via cflib (send_extpos)
  - Controla o drone via cflib (takeoff, hover, land)

Usa a forma oficial da Bitcraze para comunicação com o Crazyflie.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator


class CflibControllerNode(Node):

    def __init__(self):
        super().__init__('cflib_controller')

        # --- Parâmetros ---
        self.declare_parameter('robot_name',       'cf231')
        self.declare_parameter('uri',              'radio://0/80/2M/E7E7E7E7E7')
        self.declare_parameter('target_height',     1.0)
        self.declare_parameter('takeoff_duration',  3.0)
        self.declare_parameter('hover_duration',    5.0)
        self.declare_parameter('land_duration',     3.0)
        self.declare_parameter('extpos_rate',       50.0)

        robot_name          = self.get_parameter('robot_name').get_parameter_value().string_value
        self.uri            = self.get_parameter('uri').get_parameter_value().string_value
        self.target_height  = self.get_parameter('target_height').get_parameter_value().double_value
        self.takeoff_duration = self.get_parameter('takeoff_duration').get_parameter_value().double_value
        self.hover_duration   = self.get_parameter('hover_duration').get_parameter_value().double_value
        self.land_duration    = self.get_parameter('land_duration').get_parameter_value().double_value
        extpos_rate         = self.get_parameter('extpos_rate').get_parameter_value().double_value

        # --- Posição actual do drone (actualizada pelo MoCap) ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.pose_received = False
        self.pose_lock = threading.Lock()

        # --- Subscrição à pose do drone ---
        self.sub_pose = self.create_subscription(
            PoseStamped,
            f'/{robot_name}/pose',
            self.pose_callback,
            10
        )

        # --- Timer para enviar extpos ao drone ---
        self.timer_extpos = self.create_timer(
            1.0 / extpos_rate,
            self.extpos_callback
        )

        # --- Inicializa o cflib ---
        cflib.crtp.init_drivers()
        self.cf = None
        self.scf = None

        self.get_logger().info(
            f'CflibControllerNode iniciado:\n'
            f'  robot_name    : {robot_name}\n'
            f'  uri           : {self.uri}\n'
            f'  target_height : {self.target_height} m\n'
            f'  hover_duration: {self.hover_duration} s\n'
            f'  extpos_rate   : {extpos_rate} Hz'
        )

        # --- Lança o voo numa thread separada ---
        self.flight_thread = threading.Thread(target=self.run_flight)
        self.flight_thread.daemon = True
        self.flight_thread.start()

    def pose_callback(self, msg: PoseStamped):
        """Recebe a pose do MoCap e actualiza a posição actual."""
        with self.pose_lock:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_z = msg.pose.position.z
            self.pose_received = True

        self.get_logger().debug(
            f'[cflib_controller] Pose recebida: '
            f'x={self.current_x:.3f}, '
            f'y={self.current_y:.3f}, '
            f'z={self.current_z:.3f}'
        )

    def extpos_callback(self):
        """Envia a posição actual ao drone via send_extpos."""
        if self.cf is None:
            return
        if not self.pose_received:
            return

        with self.pose_lock:
            x = self.current_x
            y = self.current_y
            z = self.current_z

        self.cf.extpos.send_extpos(x, y, z)
        self.get_logger().debug(
            f'[cflib_controller] extpos enviado: '
            f'x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )

    def run_flight(self):
        """Corre numa thread separada — liga ao drone e executa o voo."""
        self.get_logger().info(f'[cflib_controller] A ligar ao drone em {self.uri}...')

        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            self.cf = scf.cf
            self.get_logger().info('[cflib_controller] Ligado ao drone!')

            # Espera por pose do MoCap
            self.get_logger().info('[cflib_controller] A aguardar pose do MoCap...')
            while not self.pose_received:
                time.sleep(0.1)
            self.get_logger().info('[cflib_controller] Pose recebida — a resetar estimador...')

            # Reinicia o filtro de Kalman
            reset_estimator(scf)
            self.get_logger().info('[cflib_controller] Estimador reiniciado!')

            # ARM
            self.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            # Takeoff
            self.get_logger().info(
                f'[cflib_controller] Takeoff — a subir para {self.target_height} m...'
            )
            self.do_takeoff()

            # Hover
            self.get_logger().info(
                f'[cflib_controller] A pairar durante {self.hover_duration} s...'
            )
            self.do_hover()

            # Land
            self.get_logger().info('[cflib_controller] A aterrar...')
            self.do_land()

            self.get_logger().info('[cflib_controller] Missão concluída!')
            self.cf = None

    def do_takeoff(self):
        """Sobe gradualmente até à altura alvo."""
        steps = int(self.takeoff_duration / 0.1)
        vz = self.target_height / self.takeoff_duration

        for _ in range(steps):
            self.cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(0.1)

    def do_hover(self):
        """Paira na posição actual durante hover_duration segundos."""
        with self.pose_lock:
            x = self.current_x
            y = self.current_y

        steps = int(self.hover_duration / 0.1)
        for _ in range(steps):
            self.cf.commander.send_position_setpoint(
                x, y, self.target_height, 0
            )
            time.sleep(0.1)

    def do_land(self):
        """Desce gradualmente até ao chão."""
        steps = int(self.land_duration / 0.1)
        vz = -self.target_height / self.land_duration

        for _ in range(steps):
            self.cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(0.1)

        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = CflibControllerNode()
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
