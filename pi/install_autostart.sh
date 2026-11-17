#!/bin/bash
# this makes beacon_vision.py start by itself when the pi boots, the demo has no keyboard or laptop
# bash install_autostart.sh          install it and start it now
# bash install_autostart.sh remove   uninstall it
# systemctl status demo2-vision shows if its running, journalctl -u demo2-vision -f shows the output
# stop it before running anything else that uses the camera: sudo systemctl stop demo2-vision
set -e
NAME=demo2-vision
UNIT=/etc/systemd/system/$NAME.service
if [ "$1" = "remove" ]; then
    sudo systemctl disable --now $NAME 2>/dev/null || true
    sudo rm -f "$UNIT"; sudo systemctl daemon-reload
    echo "removed"; exit 0
fi
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-$(command -v python3)}"
USER_NAME="${SUDO_USER:-$USER}"
# make sure this python has opencv and smbus2 before installing
"$PY" -c "import cv2, smbus2" || { echo "$PY cant import cv2 and smbus2, run with PYTHON=/path/to/python3"; exit 1; }
sudo tee "$UNIT" >/dev/null <<UNIT
[Unit]
Description=EENG350 Demo 2 beacon vision
After=multi-user.target

[Service]
User=$USER_NAME
WorkingDirectory=$DIR
ExecStart="$PY" "$DIR/beacon_vision.py" --headless
Restart=always
RestartSec=2
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable --now $NAME
echo "installed, it starts on every boot now"
systemctl --no-pager status $NAME | head -5
