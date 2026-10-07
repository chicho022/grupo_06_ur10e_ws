# Cinemática directa e inversa del UR10e con ROS 2 Jazzy

**IMT-342 Robótica — Universidad Católica Boliviana "San Pablo"**
Primer Parcial Práctico · **Grupo 6**
Autores: **Sebastián Ariñez** y **Marco Jaldín**
Docente: Carlos Daniel Aguilar Mancachi

Robot asignado: **Universal Robots UR10e** (6 GDL, todas las articulaciones rotacionales).

---

## 1. Objetivo

Modelar el UR10e con la convención **Denavit–Hartenberg estándar**, obtener su **cinemática
directa** (posición y orientación del efector `tool0`) y su **Jacobiano posicional**, y
resolver la **cinemática inversa de posición** con un método iterativo basado en la
pseudoinversa del Jacobiano. Todo se implementa en nodos propios de ROS 2 y se valida
contra la transformación `base -> tool0` que publica TF a partir del URDF oficial.

## 2. Software requerido

| Componente | Versión |
| --- | --- |
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Python | 3.12 (el del sistema) |
| Middleware | `rmw_cyclonedds_cpp` |
| Paquete oficial del robot | `Universal_Robots_ROS2_Description` (rama jazzy, commit fijado en `dependencias.repos`) |
| Librerías Python | NumPy (nodos), SymPy (solo `scripts/dh_sympy.py`) |

Paquetes de ROS usados: `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`,
`xacro`, `tf2_ros`.

## 3. Instalación desde cero

> Requisito previo: tener ROS 2 Jazzy instalado en `/opt/ros/jazzy`.
> Si usas conda, ejecuta `conda deactivate` antes de instalar o compilar.

```bash
git clone https://github.com/chicho022/grupo_06_ur10e_ws.git
cd grupo_06_ur10e_ws
chmod +x instalar.sh
./instalar.sh
```

`instalar.sh` hace todo lo necesario:

1. Comprueba que ROS 2 Jazzy esté instalado y que no haya un entorno de conda activo.
2. Instala con `apt` las herramientas (`colcon`, `rosdep`, `vcstool`) y las dependencias
   (`xacro`, `rviz2`, `robot_state_publisher`, `joint_state_publisher_gui`, `tf2_ros`,
   CycloneDDS, NumPy, SymPy).
3. Inicializa y actualiza `rosdep`.
4. **Recupera el paquete oficial del robot** con `vcs import src < dependencias.repos`.
   `ur_description` se descarga en `src/ur_description`, fijado al commit
   `6b639c2efd9f4f12da858a72cf1a9e40da365ed8`. No se versiona en este repositorio.
5. Instala las dependencias declaradas en los `package.xml` con `rosdep`.
6. Compila con `colcon build --symlink-install`.

### Instalación manual (equivalente)

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
mkdir -p src
vcs import src < dependencias.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 4. Uso

En **cada terminal nueva**, desde la raíz del repositorio:

```bash
source entorno.sh
```

`entorno.sh` carga ROS 2 Jazzy, selecciona CycloneDDS y carga `install/setup.bash`.

### 4.1 Abrir el robot en RViz2

Con los sliders del Joint State Publisher GUI (para mover el robot a mano y probar la FK):

```bash
ros2 launch grupo06_ur10e_bringup display.launch.py
```

Sin el GUI (obligatorio para probar la IK, porque `ik_node` publica en `/joint_states`):

```bash
ros2 launch grupo06_ur10e_bringup display.launch.py gui:=false
```

### 4.2 Cinemática directa

```bash
ros2 run grupo06_ur10e_kinematics fk_node
```

Se suscribe a `/joint_states`, calcula T06 con las matrices DH propias y muestra, una vez
por segundo y solo cuando q cambia: q, posición, cuaternión, R06 y el error contra TF.

### 4.3 Cinemática inversa

```bash
ros2 run grupo06_ur10e_kinematics ik_node
```

En otra terminal, enviar un objetivo cartesiano (en metros, frame `base`):

```bash
ros2 topic pub --once /target geometry_msgs/msg/Point "{x: 0.40, y: 0.20, z: 0.50}"
```

## 5. Ejemplo completo de prueba (IK)

| Terminal | Comando |
| --- | --- |
| 1 | `source entorno.sh` y `ros2 launch grupo06_ur10e_bringup display.launch.py gui:=false` |
| 2 | `source entorno.sh` y `ros2 run grupo06_ur10e_kinematics ik_node` |
| 3 | `source entorno.sh` y `ros2 run grupo06_ur10e_kinematics fk_node` (verificación, opcional) |
| 4 | `source entorno.sh` y `ros2 topic pub --once /target geometry_msgs/msg/Point "{x: -0.5, y: -0.3, z: 0.6}"` |

Salida esperada en la terminal 2 (resumida):

```
objetivo p_d [m]  = [-0.5 -0.3  0.6]
q0 [rad]          = [ 0.     -1.5708  1.5708 -1.5708 -1.5708  0.    ]
q* [rad]          = [ 0.2326 -1.7497  1.8305 -1.4570 -1.5496  0.    ]
iteraciones       = 12
p alcanzada [m]   = [-0.5 -0.3  0.6]
error final [m]   = 6.05e-05
CONVERGIÓ: se publica q* en /joint_states
```

El robot se mueve en RViz2 y el `fk_node` confirma la posición con error ~0 contra TF.

## 6. Tópicos, mensajes y parámetros

| Nodo | Suscribe | Publica |
| --- | --- | --- |
| `fk_node` | `/joint_states` (`sensor_msgs/msg/JointState`) | — (salida por consola; consulta TF `base -> tool0` solo como referencia) |
| `ik_node` | `/target` (`geometry_msgs/msg/Point`) | `/joint_states` (`sensor_msgs/msg/JointState`, 10 Hz) |

Parámetros del `ik_node`:

| Parámetro | Por defecto | Descripción |
| --- | --- | --- |
| `alpha` | `0.5` | Factor de actualización: q ← q + alpha·J⁺·e |
| `tolerancia` | `1e-4` | Criterio de convergencia ‖p_d − p(q)‖ < tolerancia [m] |
| `max_iter` | `300` | Máximo de iteraciones |
| `paso_max` | `0.2` | Máximo cambio por articulación e iteración [rad] |
| `q0` | `[0, -π/2, π/2, -π/2, -π/2, 0]` | Semilla (configuración inicial) [rad] |

Ejemplo de cambio de parámetros:

```bash
ros2 run grupo06_ur10e_kinematics ik_node --ros-args -p alpha:=1.0 -p tolerancia:=1e-5
```

Argumentos del launch `display.launch.py`: `gui` (`true`/`false`), `ur_type` (por defecto
`ur10e`) y `rviz_config`.

## 7. Modelo cinemático (resumen)

Convención: A_i = Rz(θ_i)·Tz(d_i)·Tx(a_i)·Rx(α_i). Marco base: `base`; efector: `tool0`.

| i | θ_i | d_i [m] | a_i [m] | α_i [rad] |
| --- | --- | --- | --- | --- |
| 1 | q1 | 0.1807 | 0 | π/2 |
| 2 | q2 | 0 | −0.6127 | 0 |
| 3 | q3 | 0 | −0.57155 | 0 |
| 4 | q4 | 0.17415 | 0 | π/2 |
| 5 | q5 | 0.11985 | 0 | −π/2 |
| 6 | q6 | 0.11655 | 0 | 0 |

`base_link` está rotado Rz(π) respecto de `base` (transformación fija del URDF).
Desarrollo completo en el informe (`docs/`) y en `scripts/dh_sympy.py`.

## 8. Validación

- **FK:** 4 configuraciones (incluida una con ángulos grandes en las 6 articulaciones):
  error de posición y orientación < 1e-6 contra TF. Ver `docs/validacion_fk.md`.
- **IK:** 4 objetivos alcanzables convergen en 12–46 iteraciones con error < 1e-4 m; un
  objetivo fuera de alcance se reporta como no convergente. Ver `docs/validacion_ik.md`.
- **Pruebas unitarias del modelo** (`ur10e_dh.py`), sin necesidad de ROS:

  ```bash
  python3 src/grupo06_ur10e_kinematics/test/test_ur10e_dh.py
  ```

## 9. Estructura del repositorio

```
grupo_06_ur10e_ws/
├── src/
│   ├── grupo06_ur10e_bringup/          # launch y configuración de RViz2 (ament_cmake)
│   │   ├── launch/display.launch.py
│   │   ├── rviz/ur10e.rviz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── grupo06_ur10e_kinematics/       # nodos propios (ament_python)
│       ├── grupo06_ur10e_kinematics/
│       │   ├── ur10e_dh.py             # tabla DH, FK, Jacobiano, cuaternión
│       │   ├── fk_node.py
│       │   └── ik_node.py
│       ├── test/test_ur10e_dh.py
│       ├── resource/, package.xml, setup.py, setup.cfg, LICENSE
├── scripts/dh_sympy.py                 # derivación simbólica (SymPy) para el informe
├── docs/                               # informe PDF, validaciones y capturas
│   ├── img/
│   ├── validacion_fk.md
│   └── validacion_ik.md
├── dependencias.repos                  # ur_description fijado a un commit
├── instalar.sh
├── entorno.sh
├── requirements.txt
├── README.md
└── .gitignore
```

`build/`, `install/`, `log/` y `src/ur_description/` no se versionan (`.gitignore`).

## 10. Errores conocidos y consideraciones

- **Dos publicadores en `/joint_states`:** el GUI de sliders y `ik_node` publican en el
  mismo tópico. Para probar la IK, lanzar con `gui:=false`; si no, el robot parpadea
  entre ambas poses.
- **Varias computadoras en la misma red:** ROS 2 descubre automáticamente los nodos de
  otras máquinas con el mismo `ROS_DOMAIN_ID` (0 por defecto). Si dos integrantes trabajan
  en la misma red, cada uno debe usar un dominio distinto, por ejemplo en `~/.bashrc`:
  `export ROS_DOMAIN_ID=61`.
- **Sello de tiempo en `/joint_states`:** si se publican estados a mano con
  `ros2 topic pub`, usar `header: auto`; con sello 0, `robot_state_publisher` no publica TF.
- **Conda:** el entorno `(base)` de conda interfiere con las herramientas de ROS 2.
  Ejecutar `conda deactivate` antes de instalar o compilar.
- **Alcance:** objetivos a más de ~1.3 m del hombro no son alcanzables; `ik_node` lo
  informa con un WARN y mantiene la pose anterior.
- La IK es solo de posición: `q6` (wrist_3) no afecta la posición de `tool0` y conserva
  el valor de la semilla.

## 11. Entrega

- Rama de entrega: `main`
- Commit evaluado: `7be7932ae0fc35a3f650d9c0e3035e321663679b`
- Informe: `docs/IMT342_arinez_jaldin.pdf`

## Licencia

MIT.
