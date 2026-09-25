#!/usr/bin/env bash
# Install the AMR controller on the robot's Raspberry Pi 5.
#
#   sudo deploy/pi/install.sh
#
# Needs Ster-Vis installed first (its deploy/pi/install.sh) and calibrated.
# Installs into /opt/amr, reads /etc/amr/amr.toml, runs as an "amr" user in
# the gpio group. The service is installed but NOT started or enabled: check
# the wiring with `amr motor-test` first, then `sudo systemctl start amr`.
#
# Safe to run again: it upgrades the package and keeps your settings.
set -euo pipefail

PREFIX=/opt/amr
CONFIG=/etc/amr
UNIT=/etc/systemd/system/amr.service
SERVICE_USER=amr

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

echo "==> System packages"
apt-get update -qq
# gpiozero and lgpio from apt: lgpio is how gpiozero reaches the Pi 5's GPIO.
apt-get install -y -qq --no-install-recommends python3-venv python3-pip python3-gpiozero python3-lgpio git

if ! id "$SERVICE_USER" >/dev/null 2>&1; then
    echo "==> Creating the $SERVICE_USER user"
    useradd --system --no-create-home --shell /usr/sbin/nologin "$SERVICE_USER"
fi
getent group gpio >/dev/null || groupadd --system gpio
usermod -a -G gpio "$SERVICE_USER"

echo "==> Installing into $PREFIX"
mkdir -p "$PREFIX"
# System site packages, so the venv sees apt's gpiozero and lgpio.
[ -x "$PREFIX/venv/bin/python" ] || python3 -m venv --system-site-packages "$PREFIX/venv"
"$PREFIX/venv/bin/pip" install --quiet --upgrade pip
"$PREFIX/venv/bin/pip" install --quiet --upgrade "$REPO"

mkdir -p "$CONFIG"
if [ ! -f "$CONFIG/amr.toml" ]; then
    install -m 0644 "$HERE/amr.toml" "$CONFIG/amr.toml"
    echo "==> Wrote $CONFIG/amr.toml"
else
    echo "==> Kept your $CONFIG/amr.toml"
fi
install -m 0644 "$HERE/amr.service" "$UNIT"
ln -sf "$PREFIX/venv/bin/amr" /usr/local/bin/amr
systemctl daemon-reload

cat <<EOF

Installed. Next:
  1. Ster-Vis settings for this robot:
       sudo cp $HERE/ster-vis.env /etc/ster-vis/ster-vis.env && sudo systemctl restart ster-vis
  2. Wheels off the ground, then check every motor turns forward:
       sudo amr motor-test
  3. Try the loop without driving:   sudo amr run --dry-run
  4. Drive:                          sudo systemctl start amr     (stop: sudo systemctl stop amr)
     Logs:                           journalctl -u amr -f
EOF
