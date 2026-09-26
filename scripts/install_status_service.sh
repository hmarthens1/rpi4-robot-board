#!/bin/bash
# =============================================================================
# Build the robot_status ROS 2 package (C++) and run its two nodes at boot
# =============================================================================
# Needs: ROS 2 Humble (rpi4-ros2-multirobot Lab 02) and robot_board
# (sudo bash scripts/install.sh).
#
#   1. installs the C++ build dependencies (apt), links ros2/robot_status into
#      ~/ros2_ws/src and builds it with colcon (Release), running its tests
#   2. writes two systemd services, both in the namespace /<hostname> and with
#      the ROS_DOMAIN_ID from your ~/.bashrc:
#        robot-status   status_node  (battery, sonar, system)  as your user
#        robot-command  command_node (motors, servos, LEDs...) as root: the
#                       RGB LEDs need /dev/mem
#   3. starts them now and at every boot
#
# USAGE (from the repo folder, on the Pi)
#   sudo bash scripts/install_status_service.sh
#   sudo bash scripts/install_status_service.sh --remove
#   journalctl -u robot-status -f          # logs
#   journalctl -u robot-command -f
# =============================================================================

set -u
say()  { echo -e "\n\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
die()  { echo -e "\n\033[1;31mERROR:\033[0m $*\n" >&2; exit 1; }

UNITS="robot-status robot-command"
[ "$(id -u)" -eq 0 ] || die "Must run with sudo:  sudo bash $0"
REAL_USER=${SUDO_USER:-}
[ -n "$REAL_USER" ] && [ "$REAL_USER" != "root" ] || die "Run it with sudo from your normal user, not as root."
REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)
REPO=$(cd "$(dirname "$0")/.." && pwd)
WS="$REAL_HOME/ros2_ws"

if [ "${1:-}" = "--remove" ]; then
  for u in $UNITS; do
    systemctl disable --now "$u.service" 2>/dev/null
    rm -f "/etc/systemd/system/$u.service"
  done
  systemctl daemon-reload
  ok "robot-status and robot-command services removed (the built package stays in $WS)"
  exit 0
fi

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble is not installed (rpi4-ros2-multirobot Lab 02)"
DOMAIN=$(grep -E '^\s*export ROS_DOMAIN_ID=' "$REAL_HOME/.bashrc" | tail -n1 | cut -d= -f2)
[ -n "$DOMAIN" ] || die "No ROS_DOMAIN_ID in $REAL_HOME/.bashrc (rpi4-ros2-multirobot Lab 02)"

say "1/3  Build robot_status (C++) in $WS"
while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do sleep 10; done
DEBIAN_FRONTEND=noninteractive apt-get install -y -q git build-essential cmake pkg-config \
  libgpiod-dev nlohmann-json3-dev libi2c-dev ros-humble-ament-cmake-gtest >/dev/null \
  || die "apt install of the build dependencies failed"
sudo -u "$REAL_USER" mkdir -p "$WS/src"
sudo -u "$REAL_USER" ln -sfn "$REPO/ros2/robot_status" "$WS/src/robot_status"
# The package used to be Python (ament_python); colcon can't switch build type in place.
if [ -f "$WS/build/robot_status/setup.py" ] || [ -d "$WS/build/robot_status/robot_status" ]; then
  rm -rf "$WS/build/robot_status" "$WS/install/robot_status"
fi
sudo -u "$REAL_USER" bash -c "source /opt/ros/humble/setup.bash && cd '$WS' && \
  colcon build --packages-select robot_status --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  colcon test --packages-select robot_status --ctest-args -R test_board && \
  colcon test-result --test-result-base build/robot_status" \
  > /tmp/robot_status_build.log 2>&1 || die "colcon build or tests failed - see /tmp/robot_status_build.log"
ok "built, $(grep -oE '[0-9]+ tests, [0-9]+ errors, [0-9]+ failures' /tmp/robot_status_build.log | tail -1)"

say "2/3  systemd services"
write_unit() {   # name, description, node, user
  cat > "/etc/systemd/system/$1.service" <<EOF
[Unit]
Description=$2
After=network-online.target
Wants=network-online.target

[Service]
User=$4
Environment=ROS_DOMAIN_ID=$DOMAIN
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 run robot_status $3 --ros-args -r __ns:=/\$(hostname)'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
}
write_unit robot-status  "ROS 2 robot_status node (battery, sonar, system status)" status_node "$REAL_USER"
write_unit robot-command "ROS 2 command node (motors, servos, LEDs, buzzer)"      command_node root
systemctl daemon-reload
for u in $UNITS; do
  systemctl enable "$u.service" >/dev/null 2>&1
  systemctl restart "$u.service"
done
ok "robot-status, robot-command (ROS_DOMAIN_ID=$DOMAIN, namespace /$(hostname))"

say "3/3  Check"
sleep 8
for u in $UNITS; do
  systemctl is-active --quiet "$u.service" && ok "$u running" || die "$u not running - journalctl -u $u -e"
  journalctl -u "$u" -n 3 --no-pager -o cat | grep -v "^Started" | sed 's/^/      /'
done
echo
echo "Status:   /$(hostname)/battery, /$(hostname)/system (+ /$(hostname)/sonar/range with a sonar)"
echo "Commands: /$(hostname)/command in, /$(hostname)/command_result out, /$(hostname)/cmd_vel"
echo "Logs:     journalctl -u robot-status -f ; journalctl -u robot-command -f"
echo
