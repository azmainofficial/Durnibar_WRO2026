#!/bin/bash
# =============================================================
# setup_hotspot.sh  –  Configure Pi as a WiFi Hotspot fallback.
#
# Run ONCE on the Raspberry Pi after first boot:
#   bash ~/pi_code/setup_hotspot.sh
#
# After setup, the Pi will:
#   - Try to connect to known WiFi networks first
#   - If no known network found within 20s → broadcast its OWN hotspot
#   - SSID: WRO-Robot | Password: wro12345
#   - Dashboard will be at: http://10.42.0.1:5000
# =============================================================

set -e

SSID="WRO-Robot"
PASSWORD="wro12345"
HOTSPOT_IFACE="wlan0"

echo "============================================"
echo "  WRO Robot WiFi Hotspot Setup"
echo "============================================"

# Check NetworkManager is installed
if ! command -v nmcli &> /dev/null; then
    echo "[ERROR] NetworkManager not found. Install with:"
    echo "  sudo apt install network-manager"
    exit 1
fi

echo "[1/3] Creating hotspot profile: '$SSID'..."
# Delete existing profile if it exists
nmcli connection delete "$SSID" 2>/dev/null || true

# Create a hotspot (access point) connection profile
sudo nmcli connection add \
    type wifi \
    ifname "$HOTSPOT_IFACE" \
    con-name "$SSID" \
    autoconnect no \
    ssid "$SSID" \
    -- \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD" \
    wifi.mode ap \
    ipv4.method shared \
    ipv4.addresses 10.42.0.1/24

echo "[2/3] Installing hotspot auto-fallback service..."
sudo tee /etc/systemd/system/wro_hotspot.service > /dev/null << 'EOF'
[Unit]
Description=WRO Robot WiFi Hotspot Fallback
After=NetworkManager-wait-online.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=root
ExecStart=/bin/bash -c '\
  sleep 20; \
  if ! nmcli -t -f STATE general | grep -q "connected"; then \
    echo "[WRO-HOTSPOT] No network found. Starting WiFi Hotspot..."; \
    nmcli connection up "WRO-Robot"; \
  else \
    echo "[WRO-HOTSPOT] Network OK. Hotspot not needed."; \
  fi'
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "[3/3] Enabling hotspot fallback service..."
sudo systemctl daemon-reload
sudo systemctl enable wro_hotspot.service

echo ""
echo "============================================"
echo "  Setup Complete!"
echo ""
echo "  On next boot:"
echo "    - If known WiFi found  -> connects normally"
echo "      Dashboard: http://192.168.137.137:5000"
echo ""
echo "    - If NO WiFi found     -> Pi creates hotspot"
echo "      Connect phone/PC to: '$SSID'"
echo "      Password: '$PASSWORD'"
echo "      Dashboard: http://10.42.0.1:5000"
echo "============================================"
