#!/usr/bin/env bash
# Instalación completa del proyecto en una computadora con Ubuntu 24.04 + ROS 2 Jazzy.
# Uso (desde la raíz del repositorio clonado):
#   chmod +x instalar.sh
#   ./instalar.sh
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

echo "== [1/6] Comprobando ROS 2 Jazzy"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: ROS 2 Jazzy no está instalado en /opt/ros/jazzy."
  echo "Instálalo siguiendo https://docs.ros.org/en/jazzy/Installation.html"
  exit 1
fi
if [ -n "${CONDA_PREFIX:-}" ]; then
  echo "ERROR: hay un entorno de conda activo ($CONDA_PREFIX)."
  echo "Ejecuta 'conda deactivate' y vuelve a correr este script."
  exit 1
fi
source /opt/ros/jazzy/setup.bash

echo "== [2/6] Instalando herramientas y dependencias del sistema (apt)"
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  python3-sympy \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf2-ros \
  ros-jazzy-rmw-cyclonedds-cpp

echo "== [3/6] Inicializando rosdep"
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

echo "== [4/6] Descargando el paquete oficial del UR (dependencias.repos, commit fijado)"
mkdir -p src
vcs import src --skip-existing < dependencias.repos

echo "== [5/6] Instalando dependencias declaradas en los package.xml (rosdep)"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

echo "== [6/6] Compilando el workspace"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
chmod +x entorno.sh

echo
echo "INSTALACIÓN COMPLETA - Grupo 6, Universal Robots UR10e"
echo "En cada terminal nueva:"
echo "  cd $WS_DIR"
echo "  source entorno.sh"
echo "Abrir el robot:"
echo "  ros2 launch grupo06_ur10e_bringup display.launch.py"
