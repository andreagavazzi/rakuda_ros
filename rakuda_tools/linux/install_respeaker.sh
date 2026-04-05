#!/usr/bin/env bash
# install_respeaker_led_off.sh
# Installs ReSpeaker LED-off service for Rakuda on Jetson Orin (Ubuntu/Debian)
# Usage: sudo bash install_respeaker_led_off.sh

set -euo pipefail

# ── colours ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── must run as root ────────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || error "Run with sudo: sudo bash $0"

INSTALL_DIR="/opt/rakuda"
PYTHON_BIN="$(which python3)"
SERVICE_NAME="respeaker-led-off"
UDEV_RULE="/etc/udev/rules.d/60-respeaker.rules"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_FILE="${INSTALL_DIR}/led_off.py"

# ── 1. pip install pixel-ring ───────────────────────────────────────────────────
info "Installing pixel-ring and pyusb via pip..."
"$PYTHON_BIN" -m pip install --quiet pyusb pixel-ring --break-system-packages \
  || error "pip install failed"
info "pixel-ring installed."

# ── 2. udev rule ───────────────────────────────────────────────────────────────
info "Writing udev rule → ${UDEV_RULE}"
cat > "$UDEV_RULE" <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
EOF
chmod 644 "$UDEV_RULE"
info "udev rule written."

# ── 3. python script ───────────────────────────────────────────────────────────
info "Creating install directory → ${INSTALL_DIR}"
mkdir -p "$INSTALL_DIR"

info "Writing Python script → ${SCRIPT_FILE}"
cat > "$SCRIPT_FILE" <<'PYEOF'
#!/usr/bin/env python3
"""
led_off.py – turns off all ReSpeaker 4-mic array LEDs at boot.
Retries up to 10 seconds waiting for the USB device to enumerate.
"""
import time
import sys

try:
    import usb.core
    from pixel_ring import pixel_ring
except ImportError as e:
    print(f"[led_off] Import error: {e}", file=sys.stderr)
    sys.exit(1)

VENDOR_ID  = 0x2886
PRODUCT_ID = 0x0018
MAX_RETRIES = 10

for attempt in range(MAX_RETRIES):
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is not None:
        try:
            pixel_ring.off()
            print(f"[led_off] ReSpeaker LEDs turned off (attempt {attempt + 1})")
            sys.exit(0)
        except Exception as e:
            print(f"[led_off] pixel_ring.off() failed: {e}", file=sys.stderr)
            sys.exit(1)
    print(f"[led_off] Device not found, retrying ({attempt + 1}/{MAX_RETRIES})...")
    time.sleep(1)

print("[led_off] ReSpeaker not found after 10 attempts. Is it plugged in?",
      file=sys.stderr)
sys.exit(1)
PYEOF

chmod +x "$SCRIPT_FILE"
info "Python script written."

# ── 4. systemd service ─────────────────────────────────────────────────────────
info "Writing systemd service → ${SERVICE_FILE}"
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Turn off ReSpeaker LED ring on boot (Rakuda)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${PYTHON_BIN} ${SCRIPT_FILE}
RemainAfterExit=yes
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE_FILE"
info "Service file written."

# ── 5. reload udev + systemd, enable service ───────────────────────────────────
info "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

info "Reloading systemd daemon..."
systemctl daemon-reload

info "Enabling and starting ${SERVICE_NAME}..."
systemctl enable "${SERVICE_NAME}.service"
systemctl start  "${SERVICE_NAME}.service" || warn "Service start failed (device may not be plugged in yet — it will run at next boot)"

# ── 6. summary ─────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  ReSpeaker LED-off service installed successfully${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  Python script : ${SCRIPT_FILE}"
echo "  udev rule     : ${UDEV_RULE}"
echo "  Service       : ${SERVICE_FILE}"
echo ""
echo "  Check status  : systemctl status ${SERVICE_NAME}"
echo "  Check logs    : journalctl -u ${SERVICE_NAME} -n 20"
echo "  Manual test   : python3 ${SCRIPT_FILE}"
echo ""