# Validación de cinemática directa — UR10e (Grupo 6)

**Método.** La FK propia (`fk_node`, matrices DH de `ur10e_dh.py`) se compara con la
transformación `base -> tool0` que publica TF (`robot_state_publisher` + URDF oficial).
TF se usa solo como referencia de validación.

- Marco base: `base` (coincide con la base DH; `base_link` está rotado Rz(pi) respecto de `base`).
- Efector: `tool0`.
- Orden de q: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
- Error de posición: `||p_calc - p_ROS||`. Error de orientación: `2·arccos(|q_calc · q_ROS|)`.

## Tabla IV — Resumen

| Caso | q [rad] | p_calc [m] | p_ROS [m] | Error pos. [m] | Error ori. [rad] |
| --- | --- | --- | --- | --- | --- |
| 0 (cero) | [0, 0, 0, 0, 0, 0] | [-1.1843, -0.2907, 0.0609] | [-1.1843, -0.2907, 0.0608] | < 1e-6 | < 1e-6 |
| 1 (sliders) | [-4.2736, -1.0610, -1.6986, -4.8060, -1.3640, 0.4428] | [0.1160, 0.2190, 0.7851] | [0.116, 0.219, 0.785] * | 1.0e-4 * | ≈ 1e-3 * |
| 2 | [0.5, -1.2, 1.0, -1.4, -1.57, 0.3] | [-0.7110, -0.5870, 0.7523] | [-0.7110, -0.5870, 0.7523] | < 1e-6 | < 1e-6 |
| 3 | [-1.0, -0.8, -1.5, 0.7, 1.2, -0.5] | [-0.2700, 0.0200, 1.1585] | [-0.2700, 0.0200, 1.1585] | < 1e-6 | < 1e-6 |
| 4 | [2.0, -2.0, 2.2, -2.5, 0.6, 1.5] | [0.3918, -0.2064, 0.7532] | [0.3918, -0.2064, 0.7532] | < 1e-6 | < 1e-6 |

\* Caso 1: la referencia se tomó con `tf2_echo`, que imprime solo 3 decimales; el error
reportado corresponde a ese redondeo, no al modelo. Los casos 2–4 usan la comparación
automática del `fk_node` (precisión completa).

En el caso 0, la diferencia en z (0.0609 vs 0.0608) es solo de redondeo al mostrar 4 decimales
(valor real 0.06085 = d1 - d5); el error calculado es < 1e-6 m.

## Orientación (cuaterniones x, y, z, w)

| Caso | quat_calc | quat_ROS |
| --- | --- | --- |
| 0 | [0.7071, 0.0000, 0.0000, 0.7071] | [0.7071, 0.0000, 0.0000, 0.7071] |
| 1 | [0.0168, 0.9843, 0.1728, 0.0314] | [0.017, 0.984, 0.173, 0.031] |
| 2 | [0.6329, 0.7741, -0.0137, -0.0051] | [0.6329, 0.7741, -0.0137, -0.0051] |
| 3 | [-0.1512, -0.1064, 0.9822, -0.0321] | [-0.1512, -0.1064, 0.9822, -0.0321] |
| 4 | [-0.0489, 0.5358, 0.5843, 0.6075] | [-0.0489, 0.5358, 0.5843, 0.6075] |

## Matrices de rotación R06 calculadas

Caso 2:
```
[[-0.1988  0.9797 -0.0252]
 [ 0.9800  0.1985 -0.0147]
 [-0.0094 -0.0277 -0.9996]]
```
Caso 3:
```
[[-0.9522  0.0952 -0.2902]
 [-0.0309 -0.9753 -0.2187]
 [-0.3039 -0.1993  0.9316]]
```
Caso 4:
```
[[-0.2570 -0.7624  0.5939]
 [ 0.6576  0.3124  0.6855]
 [-0.7081  0.5668  0.4211]]
```

## Cómo reproducir los casos 2–4

Terminal 1 (sin GUI, para que no haya dos publicadores en `/joint_states`):
```
ros2 launch grupo06_ur10e_bringup display.launch.py gui:=false
```
Terminal 2:
```
ros2 run grupo06_ur10e_kinematics fk_node
```
Terminal 3 (cambiar `position` según el caso):
```
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{header: auto, name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0.5, -1.2, 1.0, -1.4, -1.57, 0.3]}"
```
`header: auto` es necesario: con sello de tiempo 0, `robot_state_publisher` no publica TF.

## Caso 1 — datos originales (tf2_echo)

```
T_ROS (tf2_echo base tool0):
-0.997  0.022  0.068  0.116
 0.044  0.940  0.339  0.219
-0.056  0.341 -0.938  0.785
 0.000  0.000  0.000  1.000
```
q completo: [-4.273603522042736, -1.0610410581420684, -1.6986263087082847, -4.805964006705923, -1.3640204123167399, 0.44281597917836635]
