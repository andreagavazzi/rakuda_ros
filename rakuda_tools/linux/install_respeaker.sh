#!/usr/bin/env bash
# install_respeaker.sh
# Installs ReSpeaker LED-off service for Rakuda on Jetson Orin (Ubuntu 24.04 / JetPack 7.x)
# Usage: sudo bash install_respeaker.sh

set -euo pipefail

# ── colours ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── must run as root ────────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || error "Run with sudo: sudo bash $0"

INSTALL_DIR="/opt/rakuda"
VENV_DIR="${INSTALL_DIR}/venv"
VENV_PYTHON="${VENV_DIR}/bin/python"
SERVICE_NAME="respeaker-led-off"
UDEV_RULE="/etc/udev/rules.d/60-respeaker.rules"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SCRIPT_FILE="${INSTALL_DIR}/led_off.py"
SYSTEMD_RUN="$(command -v systemd-run)" || error "systemd-run not found"

# ── 1. create install dir + venv ────────────────────────────────────────────────
info "Creating install directory → ${INSTALL_DIR}"
mkdir -p "$INSTALL_DIR"

if [[ ! -x "$VENV_PYTHON" ]]; then
    info "Creating virtualenv → ${VENV_DIR}"
    if ! python3 -m venv "$VENV_DIR" 2>/dev/null; then
        warn "python3-venv missing, installing via apt..."
        apt-get update -qq
        apt-get install -y python3-venv python3-full \
            || error "Could not install python3-venv"
        python3 -m venv "$VENV_DIR" || error "venv creation failed"
    fi
    info "virtualenv created."
else
    info "virtualenv already present, reusing it."
fi

# ── 2. install pixel-ring + pyusb inside the venv ───────────────────────────────
info "Installing pyusb and pixel-ring into the venv..."
"$VENV_PYTHON" -m pip install --quiet --upgrade pip
"$VENV_PYTHON" -m pip install --quiet pyusb pixel-ring \
    || error "pip install failed"
info "pixel-ring installed."

# libusb backend is a system library, not a pip package
if ! ldconfig -p | grep -q libusb-1.0; then
    info "Installing libusb-1.0-0 (pyusb backend)..."
    apt-get install -y libusb-1.0-0 || warn "libusb install failed — pyusb may not find a backend"
fi

# ── 3. python script ───────────────────────────────────────────────────────────
info "Writing Python script → ${SCRIPT_FILE}"
cat > "$SCRIPT_FILE" <<'PYEOF'
#!/usr/bin/env python3
"""
led_off.py  –  spegne i LED del ReSpeaker 4-mic array.
Chiamato sia dal service systemd all'avvio, sia dalla udev rule ad ogni plug.
udev garantisce che il device sia già enumerato; un piccolo delay
assicura che l'endpoint di controllo USB sia pronto.
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

# piccolo delay: l'endpoint di controllo USB può non essere
# ancora pronto subito dopo l'evento udev
time.sleep(0.5)

dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
if dev is None:
    print("[led_off] ReSpeaker non trovato.", file=sys.stderr)
    sys.exit(1)

try:
    pixel_ring.off()
    print("[led_off] LEDs spenti.")
    sys.exit(0)
except Exception as e:
    print(f"[led_off] Errore: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF

chmod +x "$SCRIPT_FILE"
info "Python script written."

# ── 4. udev rule ───────────────────────────────────────────────────────────────
info "Writing udev rule → ${UDEV_RULE}"
cat > "$UDEV_RULE" <<EOF
# ReSpeaker 4-mic array (USB ID 2886:0018)
# Set permissions and turn off LEDs on every plug event
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \\
    ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", \\
    MODE="0666", \\
    RUN+="${SYSTEMD_RUN} --no-block ${VENV_PYTHON} ${SCRIPT_FILE}"
EOF
chmod 644 "$UDEV_RULE"
info "udev rule written."

# ── 5. systemd service ─────────────────────────────────────────────────────────
# NOTE: Restart= is NOT allowed with Type=oneshot — systemd refuses the unit.
info "Writing systemd service → ${SERVICE_FILE}"
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Turn off ReSpeaker LED ring on boot (Rakuda)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${VENV_PYTHON} ${SCRIPT_FILE}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$SERVICE_FILE"
info "Service file written."

# ── 6. reload udev + systemd, enable service ───────────────────────────────────
info "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

info "Reloading systemd daemon..."
systemctl daemon-reload

info "Enabling and starting ${SERVICE_NAME}..."
systemctl enable "${SERVICE_NAME}.service"
systemctl start  "${SERVICE_NAME}.service" \
    || warn "Service start failed (device may not be plugged in yet — it will run at next boot)"

# ── 7. summary ─────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  ReSpeaker LED-off service installed successfully${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "  Virtualenv    : ${VENV_DIR}"
echo "  Python script : ${SCRIPT_FILE}"
echo "  udev rule     : ${UDEV_RULE}"
echo "  Service       : ${SERVICE_FILE}"
echo ""
echo "  Check status  : systemctl status ${SERVICE_NAME}"
echo "  Check logs    : journalctl -u ${SERVICE_NAME} -n 20"
echo "  Manual test   : ${VENV_PYTHON} ${SCRIPT_FILE}"
echo ""
