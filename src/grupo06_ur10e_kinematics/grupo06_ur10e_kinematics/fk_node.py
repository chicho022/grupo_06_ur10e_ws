"""Nodo de cinemática directa (FK) del UR10e.

Se suscribe a /joint_states, calcula T06 con las matrices DH propias
(ur10e_dh.py) y muestra la posición y orientación del efector.
TF (base -> tool0) se consulta solo como referencia de validación.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from grupo06_ur10e_kinematics.ur10e_dh import (
    JOINT_NAMES,
    forward_kinematics,
    rotation_to_quaternion,
)


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')

        # Último vector articular recibido y último que se imprimió
        self.q = None
        self.q_impreso = None

        # Entrada: estados articulares (GUI de sliders o ik_node)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # TF solo como referencia para comparar (NO es nuestra FK)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Se imprime 1 vez por segundo, y solo si q cambió
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('fk_node listo: escuchando /joint_states')

    def joint_states_callback(self, msg):
        # Diccionario nombre -> posición: el orden del mensaje puede variar
        posiciones = {}
        for i in range(len(msg.name)):
            posiciones[msg.name[i]] = msg.position[i]

        # Ignorar mensajes que no traen las 6 articulaciones del UR10e
        for nombre in JOINT_NAMES:
            if nombre not in posiciones:
                return

        # Vector q en el orden de la tabla DH (q1 ... q6)
        self.q = np.array([posiciones[nombre] for nombre in JOINT_NAMES])

    def timer_callback(self):
        if self.q is None:
            return
        if self.q_impreso is not None and np.allclose(self.q, self.q_impreso, atol=1e-6):
            return

        # Cinemática directa propia: T06 = A1 A2 A3 A4 A5 A6
        T = forward_kinematics(self.q)
        p = T[:3, 3]
        R = T[:3, :3]
        quat = rotation_to_quaternion(R)

        texto = [
            '',
            f'q [rad]           = {np.array2string(self.q, precision=4)}',
            f'p_calc [m]        = {np.array2string(p, precision=4)}',
            f'quat_calc (xyzw)  = {np.array2string(quat, precision=4)}',
            'R06 =',
            np.array2string(R, precision=4),
        ]

        # Referencia de validación: TF base -> tool0
        try:
            tf = self.tf_buffer.lookup_transform('base', 'tool0', Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            p_ros = np.array([t.x, t.y, t.z])
            quat_ros = np.array([r.x, r.y, r.z, r.w])

            error_pos = np.linalg.norm(p - p_ros)
            # Ángulo entre orientaciones; abs() porque q y -q son la misma rotación
            error_ori = 2.0 * np.arccos(min(1.0, abs(np.dot(quat, quat_ros))))
            self.q_impreso = self.q.copy()   # solo se marca como impreso si hubo comparación
            texto += [
                f'p_ROS (TF) [m]    = {np.array2string(p_ros, precision=4)}',
                f'quat_ROS (xyzw)   = {np.array2string(quat_ros, precision=4)}',
                f'error posición    = {error_pos:.6f} m',
                f'error orientación = {error_ori:.6f} rad',
            ]
        except TransformException:
            texto.append('TF base -> tool0 todavía no disponible (sin comparación)')

        self.get_logger().info('\n'.join(texto))


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()