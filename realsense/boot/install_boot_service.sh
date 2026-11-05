#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# install_boot_service.sh: ON THE GO2. Makes the RealSense camera node start
# at power-on (systemd), only when the D435i is plugged in.
#
#   sudo ./install_boot_service.sh            # install + enable (starts at next boot)
#   sudo ./install_boot_service.sh --hotplug  # also start when the camera is plugged in later
#   sudo ./install_boot_service.sh --uninstall
#   ./install_boot_service.sh --status
#
# Writes only these files (no ROS, Unitree or network config is touched):
#   /opt/go2-realsense/go2_realsense_boot.sh
#   /etc/systemd/system/go2-realsense.service
#   /etc/go2-realsense.env                      (kept if it already exists)
#   /etc/udev/rules.d/99-go2-realsense.rules    (only with --hotplug)
# ---------------------------------------------------------------------------
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT=/etc/systemd/system/go2-realsense.service
ENVF=/etc/go2-realsense.env
BIN=/opt/go2-realsense/go2_realsense_boot.sh
RULE=/etc/udev/rules.d/99-go2-realsense.rules

need_root() { [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }; }

case "${1:-}" in
    --status)
        systemctl is-enabled go2-realsense 2>/dev/null || true
        systemctl status go2-realsense --no-pager -l 2>/dev/null | head -20 || true
        [ -f "${RULE}" ] && echo "hotplug rule: ${RULE}"
        exit 0 ;;
    --uninstall)
        need_root
        systemctl disable --now go2-realsense 2>/dev/null || true
        rm -f "${UNIT}" "${BIN}" "${RULE}"
        rmdir /opt/go2-realsense 2>/dev/null || true
        systemctl daemon-reload
        udevadm control --reload-rules 2>/dev/null || true
        echo "removed (settings kept in ${ENVF}; delete it by hand if you want)"
        exit 0 ;;
    ""|--hotplug) ;;
    *) sed -n '2,17p' "$0"; exit 2 ;;
esac

need_root
systemctl is-enabled docker >/dev/null 2>&1 || echo "WARNING: docker.service is not enabled at boot: sudo systemctl enable docker"

install -d /opt/go2-realsense
install -m 755 "${DIR}/go2_realsense_boot.sh" "${BIN}"
install -m 644 "${DIR}/go2-realsense.service" "${UNIT}"
if [ -f "${ENVF}" ]; then
    echo "keeping existing ${ENVF}"
else
    install -m 644 "${DIR}/go2-realsense.env" "${ENVF}"
    echo "wrote ${ENVF}: check RS_DDS_PEERS (the Ubuntu machine's IP)"
fi

if [ "${1:-}" = "--hotplug" ]; then
    cat > "${RULE}" <<'EOF'
# Start the RealSense camera service when a D435i is plugged in (go2-realsense).
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", TAG+="systemd", ENV{SYSTEMD_WANTS}+="go2-realsense.service"
EOF
    udevadm control --reload-rules
    echo "hotplug rule installed: plugging the camera in starts the service"
fi

systemctl daemon-reload
systemctl enable go2-realsense
echo "enabled: the camera node starts at every boot when the D435i is plugged in."
echo "start now:  sudo systemctl start go2-realsense"
echo "logs:       journalctl -u go2-realsense -f"
