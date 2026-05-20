# qualisys_ros2

Porta ROS 2 (Jazzy) do driver Qualisys (KumarRobotics/qualisys) para ROS 1 Noetic.

## Nós disponíveis

- `qualisys_node` — liga ao QTM e publica poses dos rigid bodies
- `qualisys_odom_node` — odometria com filtro de Kalman
- `qualisys_calib_node` — calibração do sistema

## Launch files

- `qualisys.launch.py` — lança o driver principal
- `qualisys_odom.launch.py model:=cf231` — lança a odometria
- `qualisys_calib.launch.py` — lança a calibração
- `calibrate.launch.py model:=cf231` — lança driver + calibração

## Compilação

```bash
cd ~/ros2_ws
colcon build --packages-select qualisys_ros2
source install/setup.bash
```
