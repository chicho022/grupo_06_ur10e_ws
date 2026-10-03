# Validación de cinemática inversa — UR10e (Grupo 6)

**Método.** El `ik_node` recibe un objetivo en `/target` (`geometry_msgs/Point`), resuelve la IK
de posición con la pseudoinversa del Jacobiano posicional y publica q* en `/joint_states`.
La solución se verifica (1) con la FK propia dentro del `ik_node` y (2) de forma independiente
con el `fk_node`, que compara la FK DH con TF (`base -> tool0`).

**Parámetros del algoritmo** (parámetros ROS del `ik_node`):

| Parámetro | Valor | Significado |
| --- | --- | --- |
| `alpha` | 0.5 | Factor de actualización: q_{k+1} = q_k + alpha · J⁺ · e_k |
| `tolerancia` (ε) | 1e-4 m | Convergencia cuando ‖p_d − p(q_k)‖ < ε |
| `max_iter` | 300 | Máximo de iteraciones |
| `paso_max` | 0.2 rad | Máximo cambio por articulación e iteración (se escala todo Δq) |
| `q0` | [0, −π/2, π/2, −π/2, −π/2, 0] | Semilla fija ("home", codo doblado) para que cada prueba sea reproducible |

Límites articulares aplicados con `np.clip`: ±2π en todas las articulaciones, salvo el codo (±π).

## Tabla V — Pruebas de cinemática inversa

q0 = [0, −1.5708, 1.5708, −1.5708, −1.5708, 0] rad en todos los casos.

| Caso | p_d [m] | q* [rad] | Iteraciones | p alcanzada [m] | Error final [m] | Converge |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | (−0.50, −0.30, 0.60) | [0.2326, −1.7497, 1.8305, −1.4570, −1.5496, 0.0000] | 12 | (−0.5000, −0.3000, 0.6000) | 6.05e-05 | Sí |
| 2 | (0.60, −0.40, 0.30) | [2.1554, −1.3064, 1.9779, −1.5194, −0.4396, 0.0000] | 24 | (0.5999, −0.4000, 0.3000) | 6.28e-05 | Sí |
| 3 | (0.20, 0.60, 0.90) | [−2.0405, −1.6725, 1.3932, −1.5694, −2.3381, 0.0000] | 22 | (0.2000, 0.5999, 0.9000) | 7.88e-05 | Sí |
| 4 | (0.40, 0.20, 0.50) | [−2.8183, −1.9057, 2.2567, −1.1189, −3.4342, 0.0000] | 46 | (0.3999, 0.2000, 0.5000) | 7.29e-05 | Sí |
| 5 | (1.50, 0.00, 0.20) | (mejor intento) [0.1320, −3.2254, 0.1621, −2.2623, −1.3703, 0.0000] | 300 | (1.3584, −0.0188, 0.1985) | 1.43e-01 | **No** |

**Verificación independiente (fk_node vs TF):** para las soluciones de los casos 1–4,
el error de posición y de orientación entre la FK DH y TF fue < 1e-6 m y < 1e-6 rad.

## Observaciones

- **q6 = 0 en todas las soluciones.** La columna 6 de Jv es nula (el origen de `tool0` está sobre
  el eje z5), así que la pseudoinversa —solución de norma mínima— nunca mueve wrist_3: la IK
  es solo de posición y q6 conserva el valor de la semilla.
- **Caso 4 (ejemplo de la plantilla) requiere más iteraciones (46)** porque el objetivo está
  detrás de la base respecto de la semilla: la solución gira q1 ≈ −2.82 rad (≈ −161°).
  El límite `paso_max` hace que el giro sea gradual. Muestra la sensibilidad a la semilla.
- **Caso 5 no converge:** el objetivo está a ~1.5 m del hombro, fuera del alcance del UR10e
  (~1.3 m). El mejor intento estira el brazo (q3 ≈ 0.16 rad, cerca de la singularidad de codo)
  y se queda a 0.143 m del objetivo. El `ik_node` lo informa con un WARN y **no** mueve el robot.
- **Sin límite de paso** (solo alpha), en simulación previa el caso 4 no convergía con alpha = 1
  y algunas soluciones daban vueltas innecesarias (articulaciones cerca de ±2π); de ahí la
  elección alpha = 0.5 y paso_max = 0.2 rad.

## Cómo reproducir

Terminal 1 (sin GUI: el `ik_node` es el único publicador de `/joint_states`):
```
ros2 launch grupo06_ur10e_bringup display.launch.py gui:=false
```
Terminal 2:
```
ros2 run grupo06_ur10e_kinematics ik_node
```
Terminal 3 (opcional, verificación contra TF):
```
ros2 run grupo06_ur10e_kinematics fk_node
```
Terminal 4 (un objetivo por vez):
```
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: -0.5, y: -0.3, z: 0.6}"
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.6, y: -0.4, z: 0.3}"
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.2, y: 0.6, z: 0.9}"
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.40, y: 0.20, z: 0.50}"
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 1.5, y: 0.0, z: 0.2}"
```
Capturas: `docs/img/ik_caso1.png` … `ik_caso5_no_converge.png`.
