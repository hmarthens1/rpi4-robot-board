#!/bin/bash
# =============================================================================
# Install robot_board on a robot's Pi (Ubuntu Server 22.04)
# =============================================================================
#   1. apt:  pip, a C compiler (rpi_ws281x builds from source), i2c-tools,
#            python3-rpi.gpio
#   2. pip:  this package + smbus2 + rpi_ws281x, for the whole system, so both
#            "python3" and "sudo python3" can import it and "robot-board" is on PATH
#   3. check: import, and read the battery
#
# Safe to run again (it reinstalls this folder's version).
#
# USAGE (from the repo folder, on the Pi)
#   sudo bash scripts/install.sh
# =============================================================================

set -u
say()  { echo -e "\n\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
warn() { echo -e "    \033[1;33m!!\033[0m  $*"; }
die()  { echo -e "\n\033[1;31mERROR:\033[0m $*\n" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "Must run with sudo:  sudo bash $0"
REPO=$(cd "$(dirname "$0")/.." && pwd)
[ -f "$REPO/pyproject.toml" ] || die "Run it from inside the rpi4-robot-board repo"

while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  warn "apt is busy (unattended-upgrades) - waiting 15 s..."; sleep 15
done

say "1/3  System packages"
apt-get update -q >/dev/null || die "apt update failed"
DEBIAN_FRONTEND=noninteractive apt-get install -y -q \
  python3-pip python3-dev gcc i2c-tools python3-rpi.gpio >/dev/null || die "apt install failed"
ok "python3-pip, python3-dev, gcc, i2c-tools, python3-rpi.gpio"

say "2/3  robot_board (pip, system-wide)"
# Build from a copy: building in the repo as root leaves root-owned build/ and
# *.egg-info folders in your clone that you then can't delete without sudo.
BUILD=$(mktemp -d)
cp -r "$REPO/pyproject.toml" "$REPO/setup.cfg" "$REPO/README.md" "$REPO/robot_board" "$BUILD"/
pip3 install -q "$BUILD" 2>&1 | grep -v "^WARNING: Running pip as the .root. user"
rm -rf "$BUILD"
pip3 show robot-board >/dev/null 2>&1 || die "pip install failed"
ok "robot-board $(pip3 show robot-board 2>/dev/null | awk '/^Version/{print $2}'), $(command -v robot-board)"

say "3/3  Check"
(cd / && python3 -c "import robot_board, RPi.GPIO, smbus2, rpi_ws281x") || die "import failed"   # not from the repo folder
ok "imports"
if V=$(robot-board battery); then ok "board answers: battery $V"
else warn "the board didn't answer - fitted, switched ON, battery charged?"; fi

echo
echo "Next, watch the board (nothing moves):   sudo robot-board test"
echo "Wheels off the table, then:              sudo robot-board test --motors"
echo
