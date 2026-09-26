#!/bin/bash
# =============================================================================
# Build the robot_status ROS 2 node and run it at boot as a systemd service
# =============================================================================
# Needs: ROS 2 Humble (rpi4-ros2-multirobot Lab 02) and robot_board
# (sudo bash scripts/install.sh).
#
#   1. links ros2/robot_status into ~/ros2_ws/src and builds it with colcon
#   2. writes /etc/systemd/system/robot-status.service, which runs
#        ros2 run robot_status status_node --ros-args -r __ns:=/<hostname>
#      as your user, with the ROS_DOMAIN_ID from your ~/.bashrc
#   3. starts it now and at every boot
#
# USAGE (from the repo folder, on the Pi)
#   sudo bash scripts/install_status_service.sh
#   sudo bash scripts/install_status_service.sh --remove
#   journalctl -u robot-status -f          # its log
# =============================================================================

set -u
say()  { echo -e "\n\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
die()  { echo -e "\n\033[1;31mERROR:\033[0m $*\n" >&2; exit 1; }

UNIT=/etc/systemd/system/robot-status.service
[ "$(id -u)" -eq 0 ] || die "Must run with sudo:  sudo bash $0"
REAL_USER=${SUDO_USER:-}
[ -n "$REAL_USER" ] && [ "$REAL_USER" != "root" ] || die "Run it with sudo from your normal user, not as root."
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
REPO=$(cd "$(dirname "$0")/.." && pwd)
WS="$REAL_HOME/ros2_ws"

if [ "${1:-}" = "--remove" ]; then
  systemctl disable --now robot-status.service 2>/dev/null
  rm -f "$UNIT"; systemctl daemon-reload
  ok "robot-status service removed (the built package stays in $WS)"
  exit 0
fi

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble is not installed (rpi4-ros2-multirobot Lab 02)"
python3 -c "import robot_board" 2>/dev/null || die "robot_board is not installed: sudo bash scripts/install.sh"
DOMAIN=$(grep -E '^\s*export ROS_DOMAIN_ID=' "$REAL_HOME/.bashrc" | tail -n1 | cut -d= -f2)
[ -n "$DOMAIN" ] || die "No ROS_DOMAIN_ID in $REAL_HOME/.bashrc (rpi4-ros2-multirobot Lab 02)"

say "1/3  Build robot_status in $WS"
sudo -u "$REAL_USER" mkdir -p "$WS/src"
sudo -u "$REAL_USER" ln -sfn "$REPO/ros2/robot_status" "$WS/src/robot_status"
sudo -u "$REAL_USER" bash -c "source /opt/ros/humble/setup.bash && cd '$WS' && colcon build --symlink-install --packages-select robot_status" \
  > /tmp/robot_status_build.log 2>&1 || die "colcon build failed - see /tmp/robot_status_build.log"
ok "built"

say "2/3  systemd service"
cat > "$UNIT" <<EOF
[Unit]
Description=ROS 2 robot_status node (battery, sonar, system status)
After=network-online.target
Wants=network-online.target

[Service]
User=$REAL_USER
Environment=ROS_DOMAIN_ID=$DOMAIN
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 run robot_status status_node --ros-args -r __ns:=/\$(hostname)'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable robot-status.service >/dev/null 2>&1
systemctl restart robot-status.service
ok "$UNIT (ROS_DOMAIN_ID=$DOMAIN, namespace /$(hostname))"

say "3/3  Check"
sleep 6
systemctl is-active --quiet robot-status.service && ok "running" || die "not running - journalctl -u robot-status -e"
journalctl -u robot-status -n 5 --no-pager -o cat | sed 's/^/    /'
echo
echo "Topics: /$(hostname)/battery, /$(hostname)/system, and /$(hostname)/sonar/range if a sonar is plugged in"
echo "Log:    journalctl -u robot-status -f"
echo
