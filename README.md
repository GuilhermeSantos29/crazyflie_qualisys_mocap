# crazyflie_qualisys_mocap

Integração do drone **Crazyflie 2.1 Brushless** com o sistema de captura de
movimento **Qualisys (QTM)** em **ROS 2**, para voo autónomo em ambiente
interior controlado.

## Conteúdo

| Pasta | Descrição |
|-------|-----------|
| `qualisys_ros2/` | Driver ROS 2 do Qualisys — publica a pose do drone a partir do QTM. |
| `mocap_bridge/` | Ponte que recebe a pose do MoCap, valida-a e converte-a para o `crazyflie_server` e o `drone_controller`. |
| `marker_deck/` | Ficheiros do suporte de marcadores passivos (hardware). Ignorado pelo `colcon` (`COLCON_IGNORE`). |

## Como usar

Clonar para dentro do `src/` de um workspace ROS 2 e compilar:

```bash
cd ~/ros2_ws/src
git clone <url-deste-repositorio> crazyflie_qualisys_mocap
cd ~/ros2_ws
colcon build
source install/setup.bash
```

O `colcon build` constrói apenas os pacotes ROS (`qualisys_ros2` e
`mocap_bridge`); a pasta `marker_deck/` é ignorada.

O procedimento completo de execução está descrito no manual do projeto.
