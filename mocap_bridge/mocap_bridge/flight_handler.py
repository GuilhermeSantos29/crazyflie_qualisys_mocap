"""
flight_handler.py

Módulo independente de ROS que encapsula a lógica de voo.
Gere os estados do drone e calcula os waypoints da trajectória.

Para adicionar uma nova trajectória:
  1. Adiciona uma constante TRAJECTORY_<NOME> = '<nome>'
  2. Adiciona o bloco elif correspondente em _generate_waypoints()
  3. Configura trajectory: "<nome>" em params_controller.yaml

Trajectórias disponíveis:
  - square : quadrado horizontal com rotação nos cantos
  - hover  : só pairar
"""

import math
from geometry_msgs.msg import Pose


class FlightHandler:

    # Estados do drone
    LANDED  = 'landed'
    TAKEOFF = 'takeoff'
    HOVER   = 'hover'
    FLYING  = 'flying'
    LANDING = 'landing'

    # Trajectórias disponíveis
    TRAJECTORY_SQUARE = 'square'
    TRAJECTORY_HOVER  = 'hover'

    def __init__(
        self,
        target_height: float = 1.0,
        side_length:   float = 1.0,
        trajectory:    str   = 'square',
        circle_points: int   = 16
    ):
        self.target_height = target_height
        self.side_length   = side_length
        self.trajectory    = trajectory
        self.state         = self.LANDED
        self.current_pose: Pose | None = None

        self.waypoints        = self._generate_waypoints(trajectory, side_length, circle_points, target_height)
        self.current_waypoint = 0

    def _generate_waypoints(self, trajectory: str, side_length: float, circle_points: int, target_height: float):
        if trajectory == self.TRAJECTORY_SQUARE:
            return [
                (side_length, 0.0),
                (side_length, side_length),
                (0.0,         side_length),
                (0.0,         0.0),
            ]

        elif trajectory == self.TRAJECTORY_HOVER:
            return []

        else:
            raise ValueError(
                f"Trajectória desconhecida: '{trajectory}'. "
                f"Use 'square' ou 'hover'."
            )

    def update_pose(self, pose: Pose):
        self.current_pose = pose

    def is_ready_to_fly(self):
        if self.current_pose is None:
            return False
        if self.state != self.LANDED:
            return False
        return True

    def get_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            return None
        wp = self.waypoints[self.current_waypoint]
        x = wp[0]
        y = wp[1]
        z = wp[2] if len(wp) > 2 else self.target_height
        return (x, y, z)

    def advance_waypoint(self):
        self.current_waypoint += 1

    def is_trajectory_complete(self):
        if self.trajectory == self.TRAJECTORY_HOVER:
            return False
        return self.current_waypoint >= len(self.waypoints)

    def reset_trajectory(self):
        self.current_waypoint = 0

    def reached_waypoint(self):
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
        self.state = state

    def get_state(self):
        return self.state

    def get_current_height(self):
        if self.current_pose is None:
            return 0.0
        return self.current_pose.position.z
