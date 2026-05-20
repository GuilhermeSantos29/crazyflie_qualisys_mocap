"""
Lógica de processamento das posições recebidas do sistema MoCap.
"""
 
from geometry_msgs.msg import Pose	#Importa o tipo de mensagem Pose do Ros2. 
					#Pose — posição + orientação 
					#PoseStamped — Posição com timestamp e frame_id
					#PoseArray — lista de Poses (o que o MoCap publica)
					#Point — só x, y, z
					#Vector3 — vector 3D (grandeza com direção (velocidade, força e aceleração) é util quando queremos alterar o valor da velocidade)
					#Twist — velocidade linear + angular (representa o movimento completo do drone ("voa a 1m/s para a frente durante X segundos")
 
class PoseHandler:
    """
    Processa e valida as posições recebidas do motion_capture_tracking.
    """
 
    def __init__(self, robot_name: str, max_position: float = 10.0):
        self.robot_name = robot_name
        self.max_position = max_position
        self.last_pose: Pose | None = None #cria uma variável chamada last_pose que pode ser uma Pose ou None, e começa a None. Depois recebemos uma posição valida e já passa para Pose
 
    def process(self, pose: Pose) -> Pose | None:
        """
        Recebe uma Posição e verifica se é valida
        """
        if not self._is_valid(pose):
            return None
 
        self.last_pose = pose
        return pose	#se a posicao for valida guarda-a e retorna-a, senão não retorna nada
 
    def _is_valid(self, pose: Pose) -> bool:
        """
        Verifica se a posição recebida e a orientação fazem sentido ou não
        """
        p = pose.position
        q = pose.orientation
 
        # Verifica se a posição está dentro dos limites
        if (abs(p.x) > self.max_position or abs(p.y) > self.max_position or abs(p.z) > self.max_position):
            return False
 
        # Verifica se a orientação é nula, ou seja o MoCap não conseguiu calcular a rotação do drone
        if (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0):
            return False
 
        return True
 
    def get_last_pose(self) -> Pose | None:
        """
        Retorna a última pose válida recebida.
        """
        return self.last_pose
 
