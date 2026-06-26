#!/bin/bash
# Setup completo do ambiente marsupial no Intel NUC
# Pré-requisito: Ubuntu 22.04 + ROS2 Humble instalados
# Uso: bash setup_nuc.sh
set -e

MARSUPIAL_REPO="https://github.com/tatianambsantos/marsupial_ws.git"
WORKSPACE="$HOME/marsupial_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Marsupial 4WD — Setup NUC"
echo "════════════════════════════════════════════════════════"

# ── [1] Dependências apt ──────────────────────────────────────────────────────
echo ""
echo "=== [1/6] Instalando dependências apt ==="
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-depthimage-to-laserscan \
  ros-humble-tf2-ros \
  ros-humble-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-diagnostic-updater \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  pkg-config cmake build-essential

# ── [2] librealsense2 SDK ─────────────────────────────────────────────────────
echo ""
echo "=== [2/6] Instalando librealsense2 SDK (Intel apt repo) ==="
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/librealsense.pgg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/librealsense.pgg] \
  https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
if sudo apt-get install -y librealsense2-dev librealsense2-utils; then
  echo "librealsense2 instalado via apt"
  LIBREALSENSE_VERSION=$(dpkg -l librealsense2-dev | grep "^ii" | awk '{print $3}')
  echo "Versão: $LIBREALSENSE_VERSION"
else
  echo "AVISO: apt falhou — será necessário compilar librealsense do source."
  echo "  Baixe em: https://github.com/IntelRealSense/librealsense/releases"
  echo "  Depois: cd librealsense-X.X.X && mkdir build && cd build"
  echo "    cmake .. -DBUILD_EXAMPLES=OFF -DFORCE_RSUSB_BACKEND=ON"
  echo "    make -j\$(nproc) && sudo make install && sudo ldconfig"
  exit 1
fi

# ── [3] Python deps ───────────────────────────────────────────────────────────
echo ""
echo "=== [3/6] Instalando dependências Python ==="
pip3 install odrive --upgrade

# ── [4] Workspace + clone repos ───────────────────────────────────────────────
echo ""
echo "=== [4/6] Criando workspace e clonando repositórios ==="
mkdir -p "$WORKSPACE/src"

# Pacotes marsupial (seu repo)
if [ ! -d "$WORKSPACE/src/marsupial_robot_base" ]; then
  git clone "$MARSUPIAL_REPO" "$WORKSPACE/src/marsupial"
  # Move subpastas para src/ (se o repo contém os pacotes na raiz)
  mv "$WORKSPACE/src/marsupial"/* "$WORKSPACE/src/" 2>/dev/null || true
  rm -rf "$WORKSPACE/src/marsupial"
else
  echo "Pacotes marsupial já presentes, pulando clone."
fi

# realsense-ros (ROS2 wrapper oficial)
if [ ! -d "$WORKSPACE/src/realsense-ros" ]; then
  git clone --depth 1 --branch ros2-master \
    https://github.com/IntelRealSense/realsense-ros.git \
    "$WORKSPACE/src/realsense-ros"
  # Remove depend librealsense2 do package.xml (instalado via apt/source no sistema)
  sed -i 's|<depend>librealsense2</depend>|<!-- librealsense2 system install -->|g' \
    "$WORKSPACE/src/realsense-ros/realsense2_camera/package.xml"
fi

# ── [5] udev rules ────────────────────────────────────────────────────────────
echo ""
echo "=== [5/6] Configurando udev rules ==="
# RealSense
LIBREALSENSE_RULES=$(find /usr -name "99-realsense-libusb.rules" 2>/dev/null | head -1)
if [ -n "$LIBREALSENSE_RULES" ]; then
  sudo cp "$LIBREALSENSE_RULES" /etc/udev/rules.d/
else
  echo "AVISO: udev rule da RealSense não encontrada. Procure em:"
  echo "  librealsense-source/config/99-realsense-libusb.rules"
fi
# M1CT_D2 LiDAR
if [ -f "$WORKSPACE/src/m1ct_d2/sc_mini.rules" ]; then
  sudo cp "$WORKSPACE/src/m1ct_d2/sc_mini.rules" /etc/udev/rules.d/
fi
sudo udevadm control --reload-rules
sudo udevadm trigger

# ── [6] rosdep + build ────────────────────────────────────────────────────────
echo ""
echo "=== [6/6] Instalando deps ROS e compilando workspace ==="
source "$ROS_SETUP"

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -y \
  --skip-keys "librealsense2"

cd "$WORKSPACE"
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-ignore zed_components zed_ros2 zed_wrapper

# ── Source automático ─────────────────────────────────────────────────────────
SETUP_LINE="source $WORKSPACE/install/setup.bash"
if ! grep -qF "$SETUP_LINE" ~/.bashrc; then
  echo "$SETUP_LINE" >> ~/.bashrc
fi

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Setup concluído!"
echo "  Execute: source ~/.bashrc"
echo "  Teste:   ros2 launch marsupial_robot_sensors sensors.launch.py"
echo "════════════════════════════════════════════════════════"
