#!/usr/bin/env bash
# Entorno del workspace del Grupo 6 (UR10e). Uso:  source entorno.sh
# Funciona desde cualquier ruta donde se haya clonado el repositorio.

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
else
  echo "AVISO: no existe $WS_DIR/install/setup.bash. Ejecuta primero ./instalar.sh"
fi

# Nota: si dos computadoras con ROS 2 comparten la misma red, cada una debe usar un
# ROS_DOMAIN_ID distinto (p. ej. en ~/.bashrc: export ROS_DOMAIN_ID=61) para que sus
# nodos no se mezclen. Ver "Errores conocidos" en el README.
