# Validación de cinemática directa — UR10e (Grupo 6)

Referencia: TF de ROS 2 (`ros2 run tf2_ros tf2_echo base tool0`).
Marco base: `base` (coincide con la base DH; `base_link` está rotado Rz(pi) respecto a `base`).
Efector: `tool0`.

## Caso 1

q [rad] (orden URDF: pan, lift, elbow, wrist_1, wrist_2, wrist_3):
[-4.273603522042736, -1.0610410581420684, -1.6986263087082847, -4.805964006705923, -1.3640204123167399, 0.44281597917836635]

p_ROS [m]: [0.116, 0.219, 0.785]
Cuaternión ROS (x, y, z, w): [0.017, 0.984, 0.173, 0.031]

T_ROS:
-0.997  0.022  0.068  0.116
 0.044  0.940  0.339  0.219
-0.056  0.341 -0.938  0.785
 0.000  0.000  0.000  1.000

p_calc (fk_node) [m]: PENDIENTE
Error posición [m]: PENDIENTE
Error orientación [rad]: PENDIENTE

## Caso 2
PENDIENTE

## Caso 3
PENDIENTE
