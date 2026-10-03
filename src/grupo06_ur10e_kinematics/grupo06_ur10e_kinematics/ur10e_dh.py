"""Modelo cinemático del UR10e con Denavit-Hartenberg estándar.

Módulo de solo matemática (NumPy, sin ROS), compartido por fk_node e ik_node.
Convención: A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
Marco base: 'base' de ROS (base_link rotado Rz(pi)). Efector: 'tool0'.
"""
import numpy as np

# Orden de las articulaciones tal como las define el URDF.
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Tabla DH estándar del UR10e (coincide con la de Universal Robots y con el informe).
# Una fila por articulación: (offset_theta [rad], d [m], a [m], alpha [rad]);
# theta_i = q_i + offset_theta. Todos los offsets son 0: la configuración cero del URDF
# coincide con la configuración cero DH. Los x_i apuntan en sentido opuesto al avance
# del brazo (por eso a2 y a3 son negativos), igual que el eje x del frame 'base'.
DH_TABLE = [
    # offset_th  d [m]     a [m]      alpha [rad]
    (0.0,        0.1807,   0.0,       np.pi / 2),  # 1: base -> hombro
    (0.0,        0.0,     -0.6127,    0.0),        # 2: hombro -> codo (brazo)
    (0.0,        0.0,     -0.57155,   0.0),        # 3: codo -> muñeca 1 (antebrazo)
    (0.0,        0.17415,  0.0,       np.pi / 2),  # 4: muñeca 1 -> muñeca 2
    (0.0,        0.11985,  0.0,      -np.pi / 2),  # 5: muñeca 2 -> muñeca 3
    (0.0,        0.11655,  0.0,       0.0),        # 6: muñeca 3 -> tool0 (brida)
]

# Límites articulares [rad] de joint_limits.yaml, mismo orden que JOINT_NAMES.
JOINT_LIMITS_LOWER = np.array([-2 * np.pi, -2 * np.pi, -np.pi, -2 * np.pi, -2 * np.pi, -2 * np.pi])
JOINT_LIMITS_UPPER = np.array([2 * np.pi, 2 * np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi])


def dh_matrix(theta, d, a, alpha):
    """Matriz homogénea 4x4 A_i para una fila DH (ecuación 6 de la plantilla).

    A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    Columnas 1-3: ejes x_i, y_i, z_i del frame nuevo expresados en el anterior.
    Columna 4: origen O_i del frame nuevo expresado en el anterior.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])

def forward_kinematics_all(q):
    """Lista [T00, T01, ..., T06] de transformaciones acumuladas desde la base.

    T00 = identidad; T0i = T0(i-1) @ A_i  (postmultiplicación: cada A_i está
    expresada en el frame actual). Se devuelven todas porque el Jacobiano
    geométrico necesita z_{i-1} y o_{i-1} de cada frame.
    """
    T = np.eye(4)
    transforms = [T]
    for i in range(len(DH_TABLE)):
        theta_offset, d, a, alpha = DH_TABLE[i]
        theta = q[i] + theta_offset
        T = T @ dh_matrix(theta, d, a, alpha)
        transforms.append(T)
    return transforms


def forward_kinematics(q):
    """T06 (4x4) del efector (tool0) respecto del frame 'base'."""
    return forward_kinematics_all(q)[-1]


def position_jacobian(q):
    """Jacobiano posicional Jv (3x6), método geométrico.

    Columna i:  z_{i-1} x (p - o_{i-1})
    """
    T = forward_kinematics_all(q)    # [T00, T01, ..., T06]
    p = T[6][:3, 3]                  # posición del efector (o6)

    J = np.zeros((3, 6))
    for i in range(6):
        z = T[i][:3, 2]              # eje de giro de la articulación i+1
        o = T[i][:3, 3]              # origen de ese eje
        J[:, i] = np.cross(z, p - o)

    return J


def rotation_to_quaternion(R):
    """Convierte R (3x3) a cuaternión (x, y, z, w).

    Rama principal: w = 0.5*sqrt(1 + traza). Si el giro es cercano a 180°
    (w ~ 0), se calcula primero la componente mayor de la diagonal para no
    dividir entre ~0.
    """
    traza = R[0, 0] + R[1, 1] + R[2, 2]

    if traza > 0:
        s = 2.0 * np.sqrt(1.0 + traza)          # s = 4w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])   # s = 4x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])   # s = 4y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])   # s = 4z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w])