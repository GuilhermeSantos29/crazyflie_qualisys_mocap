"""
flight_handler.py
-----------------
Lógica de voo independente de ROS.
Gere o estado do drone e calcula trajectórias.

Trajectórias disponíveis (definidas em params_controller.yaml):
  - square          : quadrado horizontal
  - circle          : círculo (volta ao ponto de partida)
  - forward         : frente e trás
  - hover           : só pairar
  - square_vertical : quadrado vertical no plano YZ
  - pwm             : sinal PWM no plano YZ (onda quadrada)
  - pentagon        : pentágono no plano YZ

Modos de controlo:
  - position : envia posições absolutas (go_to)
  - velocity : envia velocidades (cmd_vel)
"""

import math
from geometry_msgs.msg import Pose


class FlightHandler:

    # Estados possíveis do drone
    LANDED  = 'landed'
    TAKEOFF = 'takeoff'
    HOVER   = 'hover'
    FLYING  = 'flying'
    LANDING = 'landing'

    # Modos de controlo
    MODE_POSITION = 'position'
    MODE_VELOCITY = 'velocity'

    # Trajectórias disponíveis
    TRAJECTORY_SQUARE          = 'square'
    TRAJECTORY_CIRCLE          = 'circle'
    TRAJECTORY_FORWARD         = 'forward'
    TRAJECTORY_HOVER           = 'hover'
    TRAJECTORY_SQUARE_VERTICAL = 'square_vertical'
    TRAJECTORY_PWM             = 'pwm'
    TRAJECTORY_PENTAGON        = 'pentagon'

    def __init__(
        self,
        target_height: float = 1.0,
        side_length:   float = 1.0,
        mode:          str   = 'position',
        velocity:      float = 0.5,
        trajectory:    str   = 'square',
        circle_points: int   = 16
    ):
        """
        Args:
            target_height: altura de voo em metros
            side_length:   tamanho do lado/raio da trajectória em metros
            mode:          'position' ou 'velocity'
            velocity:      velocidade de voo em m/s (só para modo velocity)
            trajectory:    'square', 'circle', 'forward', 'hover',
                           'square_vertical', 'pwm' ou 'pentagon'
            circle_points: número de pontos para aproximar o círculo
        """
        self.target_height = target_height
        self.side_length   = side_length
        self.mode          = mode
        self.velocity      = velocity
        self.trajectory    = trajectory
        self.state         = self.LANDED
        self.current_pose: Pose | None = None

        self.waypoints        = self._generate_waypoints(trajectory, side_length, circle_points, target_height)
        self.current_waypoint = 0

    def _generate_waypoints(self, trajectory: str, side_length: float, circle_points: int, target_height: float):
        """
        Gera a lista de waypoints para a trajectória escolhida.
        """
        if trajectory == self.TRAJECTORY_SQUARE:
            return [
                (side_length, 0.0),
                (side_length, side_length),
                (0.0,         side_length),
                (0.0,         0.0),
            ]

        elif trajectory == self.TRAJECTORY_CIRCLE:
            waypoints = []
            for i in range(circle_points):
                angle = 2 * math.pi * i / circle_points
                x = side_length * math.cos(angle)
                y = side_length * math.sin(angle)
                waypoints.append((x, y))
            # Volta ao ponto de partida
            waypoints.append((0.0, 0.0))
            return waypoints

        elif trajectory == self.TRAJECTORY_FORWARD:
            return [
                (side_length, 0.0),
                (0.0,         0.0),
            ]

        elif trajectory == self.TRAJECTORY_HOVER:
            return []

        elif trajectory == self.TRAJECTORY_SQUARE_VERTICAL:
            return [
                (0.0, side_length, target_height),
                (0.0, side_length, target_height + side_length),
                (0.0, 0.0,         target_height + side_length),
                (0.0, 0.0,         target_height),
            ]

        elif trajectory == self.TRAJECTORY_PWM:
            # Sinal PWM no plano YZ — onda quadrada
            duty_cycle = 0.5
            num_cycles = 4
            step   = side_length / num_cycles
            z_low  = target_height
            z_high = target_height + side_length

            waypoints = []
            for i in range(num_cycles):
                y_start = i * step
                y_high  = y_start + step * duty_cycle
                y_end   = y_start + step

                waypoints.append((0.0, y_start, z_low))
                waypoints.append((0.0, y_start, z_high))
                waypoints.append((0.0, y_high,  z_high))
                waypoints.append((0.0, y_high,  z_low))
                waypoints.append((0.0, y_end,   z_low))

            # Volta ao ponto de partida
            waypoints.append((0.0, 0.0, z_low))
            return waypoints

        elif trajectory == self.TRAJECTORY_PENTAGON:
            # Pentágono no plano YZ
            waypoints = []
            for i in range(5):
                angle = 2 * math.pi * i / 5 - math.pi / 2
                y = side_length * math.cos(angle)
                z = target_height + side_length + side_length * math.sin(angle)
                waypoints.append((0.0, y, z))
            # Volta ao ponto de partida
            waypoints.append(waypoints[0])
            return waypoints

        else:
            raise ValueError(f"Trajectória desconhecida: '{trajectory}'. "
                             f"Use 'square', 'circle', 'forward', 'hover', "
                             f"'square_vertical', 'pwm' ou 'pentagon'.")

    def update_pose(self, pose: Pose):
        """Actualiza a pose actual do drone."""
        self.current_pose = pose

    def is_ready_to_fly(self):
        """Verifica se o drone está pronto para voar."""
        if self.current_pose is None:
            return False
        if self.state != self.LANDED:
            return False
        return True

    def get_next_waypoint(self):
        """Retorna o próximo waypoint (x, y, z)."""
        if self.current_waypoint >= len(self.waypoints):
            return None
        wp = self.waypoints[self.current_waypoint]
        x = wp[0]
        y = wp[1]
        z = wp[2] if len(wp) > 2 else self.target_height
        return (x, y, z)

    def advance_waypoint(self):
        """Avança para o próximo waypoint."""
        self.current_waypoint += 1

    def is_trajectory_complete(self):
        """Verifica se a trajectória está completa."""
        if self.trajectory == self.TRAJECTORY_HOVER:
            return False
        if self.current_waypoint >= len(self.waypoints):
            return True
        return False

    def reset_trajectory(self):
        """Reinicia a trajectória."""
        self.current_waypoint = 0

    def get_velocity_command(self):
        """Calcula o comando de velocidade para o waypoint actual."""
        if self.current_pose is None:
            return (0.0, 0.0, 0.0)

        waypoint = self.get_next_waypoint()
        if waypoint is None:
            return (0.0, 0.0, 0.0)

        tx, ty, tz = waypoint
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        cz = self.current_pose.position.z

        dx = tx - cx
        dy = ty - cy
        dz = tz - cz

        dist = (dx**2 + dy**2 + dz**2) ** 0.5

        if dist < 0.05:
            return (0.0, 0.0, 0.0)

        vx = (dx / dist) * self.velocity
        vy = (dy / dist) * self.velocity
        vz = (dz / dist) * self.velocity

        return (vx, vy, vz)

    def reached_waypoint(self):
        """Verifica se chegou ao waypoint actual (15cm de tolerância)."""
        if self.current_pose is None:
            return False

        waypoint = self.get_next_waypoint()
        if waypoint is None:
            return False

        tx, ty, tz = waypoint
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        cz = self.current_pose.position.z

        dist = ((tx-cx)**2 + (ty-cy)**2 + (tz-cz)**2) ** 0.5
        return dist < 0.15

    def set_state(self, state):
        """Muda o estado do drone."""
        self.state = state

    def get_state(self):
        """Retorna o estado actual do drone."""
        return self.state

    def get_current_height(self):
        """Retorna a altura actual do drone."""
        if self.current_pose is None:
            return 0.0
        return self.current_pose.position.z
