"""Nodo de cinemática inversa (IK) de posición del UR10e.

Recibe un objetivo cartesiano en /target (geometry_msgs/Point), resuelve la IK
con el método iterativo de la pseudoinversa del Jacobiano posicional y publica
la solución articular en /joint_states (sensor_msgs/JointState).
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState

from grupo06_ur10e_kinematics.ur10e_dh import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    JOINT_NAMES,
    forward_kinematics,
    position_jacobian,
)


def resolver_ik(p_d, q0, alpha, tolerancia, max_iter, paso_max):
    """IK numérica de posición: q_{k+1} = q_k + alpha * J^+ * e_k.

    Devuelve (q, iteraciones, error_final, convergio).
    """
    q = np.array(q0, dtype=float)

    for k in range(max_iter):
        p = forward_kinematics(q)[:3, 3]      # posición actual (FK propia)
        e = p_d - p                           # error de posición
        error = np.linalg.norm(e)

        if error < tolerancia:
            return q, k, error, True

        J = position_jacobian(q)              # Jacobiano posicional 3x6
        delta_q = alpha * (np.linalg.pinv(J) @ e)

        # Limitar el paso: ninguna articulación se mueve más de paso_max por iteración
        mayor = np.max(np.abs(delta_q))
        if mayor > paso_max:
            delta_q = delta_q * (paso_max / mayor)

        q = q + delta_q
        q = np.clip(q, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)   # límites articulares

    p = forward_kinematics(q)[:3, 3]
    return q, max_iter, np.linalg.norm(p_d - p), False

class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        # Parámetros del algoritmo (se pueden cambiar con --ros-args -p nombre:=valor)
        self.declare_parameter('alpha', 0.5)            # factor de actualización
        self.declare_parameter('tolerancia', 1e-4)      # epsilon [m]
        self.declare_parameter('max_iter', 300)
        self.declare_parameter('paso_max', 0.2)         # [rad] por iteración y articulación
        self.declare_parameter('q0', [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])

        self.alpha = self.get_parameter('alpha').value
        self.tolerancia = self.get_parameter('tolerancia').value
        self.max_iter = self.get_parameter('max_iter').value
        self.paso_max = self.get_parameter('paso_max').value
        self.q0 = np.array(self.get_parameter('q0').value, dtype=float)

        # Estado articular que se publica (arranca en la semilla q0)
        self.q = self.q0.copy()

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Point, '/target', self.target_callback, 10)

        # Publicación continua (10 Hz): TF descarta transformaciones viejas
        self.create_timer(0.1, self.publicar_joint_states)

        self.get_logger().info(
            f'ik_node listo: esperando objetivos en /target  '
            f'(alpha={self.alpha}, tolerancia={self.tolerancia} m, '
            f'max_iter={self.max_iter}, paso_max={self.paso_max} rad)')

    def publicar_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()   # sello de tiempo real
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in self.q]
        self.pub.publish(msg)

    def target_callback(self, msg):
        p_d = np.array([msg.x, msg.y, msg.z])
        q_inicial = self.q0.copy()

        q_sol, iteraciones, error, convergio = resolver_ik(
            p_d, q_inicial, self.alpha, self.tolerancia, self.max_iter, self.paso_max)

        # Verificación con la cinemática directa
        p_final = forward_kinematics(q_sol)[:3, 3]

        texto = [
            '',
            f'objetivo p_d [m]  = {np.array2string(p_d, precision=4)}',
            f'q0 [rad]          = {np.array2string(q_inicial, precision=4)}',
            f'q* [rad]          = {np.array2string(q_sol, precision=4)}',
            f'iteraciones       = {iteraciones}',
            f'p alcanzada [m]   = {np.array2string(p_final, precision=4)}',
            f'error final [m]   = {error:.2e}',
        ]

        if convergio:
            texto.append('CONVERGIÓ: se publica q* en /joint_states')
            self.q = q_sol
            self.get_logger().info('\n'.join(texto))
        else:
            texto.append('NO CONVERGIÓ: el robot se mantiene en la pose anterior '
                         '(¿objetivo fuera del alcance?)')
            self.get_logger().warn('\n'.join(texto))


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()