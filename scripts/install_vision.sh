#!/bin/bash
# =============================================================================
# Camera vision on the robot: OpenCV, ncnn + YOLOX models, the robot_vision node
# =============================================================================
# Needs: ROS 2 Humble (rpi4-ros2-multirobot Lab 02) and the robot_status
# services (sudo bash scripts/install_status_service.sh).
#
#   1. OpenCV 4.5.4 from Ubuntu (apt) - the same OpenCV that ROS 2 Humble's
#      cv_bridge is built against. Built with V4L2, GStreamer, FFmpeg and NEON.
#   2. ncnn (Tencent's neural-network runtime for ARM CPUs), built from source at
#      a pinned release (Ubuntu 22.04 doesn't package it), static, in /usr/local,
#      without the ARMv8.2+ code paths the Pi 4's Cortex-A72 (ARMv8.0) can't run.
#   3. YOLOX nano and tiny, converted to ncnn (COCO, 80 classes), checked
#      against SHA-256 sums, in /usr/local/share/robot_vision/models
#   4. builds ros2/robot_vision in ~/ros2_ws and runs vision_node at boot as the
#      robot-vision service (your user, namespace /<hostname>)
#
# USAGE (from the repo folder, on the Pi)
#   sudo bash scripts/install_vision.sh               # everything
#   sudo bash scripts/install_vision.sh --deps-only   # steps 1-3
#   sudo bash scripts/install_vision.sh --remove      # remove the service
#   journalctl -u robot-vision -f
# =============================================================================

set -u
say()  { echo -e "\n\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
die()  { echo -e "\n\033[1;31mERROR:\033[0m $*\n" >&2; exit 1; }

NCNN_TAG=20260526
MODEL_REPO=https://raw.githubusercontent.com/Qengineering/YoloX-ncnn-Raspberry-Pi-4/3b1b307278d766f882ab85dfb01b78d954322eeb
MODELS=/usr/local/share/robot_vision/models
MODEL_SUMS="
ee80b3cb89579aedae8ade8b61eacef366b502296acee2dad082174747f25c59  yoloxN.param
5baf60130ef0accb6d1bf3cc4457e9d2b66d86f234e152d070931d6d9274fb30  yoloxN.bin
9b321794d151b878b55d3e9ee66655b32a08394e2d41dd781f5af1785b09bc25  yoloxT.param
fc87722d2228b4ee62e2d4fcc2ea20d11628b786800e1742c13b3c0a2004d405  yoloxT.bin
70507504899f8c2e5aa7dbd49fa06a38bf3478f97078e4766d1836256cddfcac  parking.jpg"

[ "$(id -u)" -eq 0 ] || die "Must run with sudo:  sudo bash $0"
REAL_USER=${SUDO_USER:-}
[ -n "$REAL_USER" ] && [ "$REAL_USER" != "root" ] || die "Run it with sudo from your normal user, not as root."
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
REPO=$(cd "$(dirname "$0")/.." && pwd)
WS="$REAL_HOME/ros2_ws"

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now robot-vision.service 2>/dev/null
  rm -f /etc/systemd/system/robot-vision.service
  systemctl daemon-reload
  ok "robot-vision service removed (OpenCV, ncnn and the models stay)"
  exit 0
fi

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble is not installed (rpi4-ros2-multirobot Lab 02)"

say "1/4  OpenCV (apt)"
while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do sleep 10; done
DEBIAN_FRONTEND=noninteractive apt-get install -y -q build-essential cmake git v4l-utils \
  libopencv-dev python3-opencv libgomp1 nlohmann-json3-dev \
  ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-ament-cmake-gtest >/dev/null \
  || die "apt install failed (try: sudo apt-get update)"
ok "OpenCV $(pkg-config --modversion opencv4)"
usermod -aG video "$REAL_USER"

say "2/4  ncnn $NCNN_TAG"
if [ -f "/usr/local/lib/cmake/ncnn/ncnnConfig.cmake" ] && grep -qs "$NCNN_TAG" /usr/local/share/ncnn_version; then
  ok "already installed"
else
  SRC=$(mktemp -d)
  git -c advice.detachedHead=false clone -q --depth 1 --branch "$NCNN_TAG" https://github.com/Tencent/ncnn.git "$SRC/ncnn" \
    || die "could not download ncnn $NCNN_TAG"
  cmake -S "$SRC/ncnn" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DNCNN_VULKAN=OFF -DNCNN_OPENMP=ON -DNCNN_BUILD_TOOLS=OFF -DNCNN_BUILD_EXAMPLES=OFF \
    -DNCNN_BUILD_TESTS=OFF -DNCNN_BUILD_BENCHMARK=OFF -DNCNN_SHARED_LIB=OFF \
    -DNCNN_ARM82=OFF -DNCNN_ARM84BF16=OFF -DNCNN_ARM84I8MM=OFF -DNCNN_ARM86SVE=OFF >/dev/null \
    || die "ncnn cmake failed"
  echo "    building (takes a while on a Pi 4)..."
  cmake --build "$SRC/build" -j"$(nproc)" >"$SRC/build.log" 2>&1 || { tail -20 "$SRC/build.log"; die "ncnn build failed"; }
  cmake --install "$SRC/build" >/dev/null || die "ncnn install failed"
  echo "$NCNN_TAG" > /usr/local/share/ncnn_version
  rm -rf "$SRC"
  ok "ncnn installed in /usr/local"
fi

say "3/4  YOLOX models"
mkdir -p "$MODELS"
while read -r sum name; do
  [ -n "$name" ] || continue
  if ! echo "$sum  $MODELS/$name" | sha256sum -c --quiet - >/dev/null 2>&1; then
    curl -fsSL -o "$MODELS/$name" "$MODEL_REPO/$name" || die "download of $name failed"
    echo "$sum  $MODELS/$name" | sha256sum -c --quiet - || die "$name: checksum mismatch"
  fi
done <<< "$MODEL_SUMS"
ok "yoloxN (nano) and yoloxT (tiny) in $MODELS"

[ "${1:-}" = "--deps-only" ] && exit 0

say "4/4  Build robot_vision and start the robot-vision service"
DOMAIN=$(grep -E '^\s*export ROS_DOMAIN_ID=' "$REAL_HOME/.bashrc" | tail -n1 | cut -d= -f2)
[ -n "$DOMAIN" ] || die "No ROS_DOMAIN_ID in $REAL_HOME/.bashrc (rpi4-ros2-multirobot Lab 02)"
sudo -u "$REAL_USER" mkdir -p "$WS/src"
sudo -u "$REAL_USER" ln -sfn "$REPO/ros2/robot_vision" "$WS/src/robot_vision"
sudo -u "$REAL_USER" bash -c "source /opt/ros/humble/setup.bash && cd '$WS' && \
  MAKEFLAGS=-j2 colcon build --packages-select robot_vision --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3 && \
  colcon test --packages-select robot_vision >/dev/null && colcon test-result --all | tail -1" \
  || die "robot_vision build failed"
[ -x "$WS/install/robot_vision/lib/robot_vision/vision_node" ] || die "vision_node was not built"

NS=$(hostname)
cat > /etc/systemd/system/robot-vision.service <<EOF
[Unit]
Description=robot_vision vision_node (camera: lane tracking, YOLOX) in /$NS
After=network-online.target robot-command.service
Wants=network-online.target

[Service]
User=$REAL_USER
Environment=ROS_DOMAIN_ID=$DOMAIN
Environment=OMP_WAIT_POLICY=PASSIVE
ExecStart=/bin/bash -c 'source $WS/install/setup.bash && exec ros2 run robot_vision vision_node --ros-args -r __ns:=/$NS -p models_dir:=$MODELS'
Restart=on-failure
RestartSec=3
Nice=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now robot-vision.service >/dev/null 2>&1
sleep 4
systemctl is-active --quiet robot-vision && ok "robot-vision running" || die "robot-vision did not start: journalctl -u robot-vision"
journalctl -u robot-vision -n 3 --no-pager -o cat
cat <<EOF

Topics:   /$NS/vision/lane, /$NS/vision/detections, /$NS/vision/image/compressed
Control:  /$NS/vision/control  (JSON, see ros2/robot_vision/README.md)
Logs:     journalctl -u robot-vision -f
EOF
