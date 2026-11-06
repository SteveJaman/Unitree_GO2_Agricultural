#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# install_boot_service.sh: ON THE GO2. Makes the RealSense camera node start
# at power-on (systemd), only when the D435i is plugged in.
# Usually run for you by:  ./scripts/tools/realsense_host.sh setup-robot
#
#   sudo ./install_boot_service.sh [--host-mac MAC] [--hotplug]
#                                     install / update + enable (starts at every boot)
#   sudo ./install_boot_service.sh --uninstall
#   ./install_boot_service.sh --status
#
#   --host-mac MAC   the workstation's Wi-Fi MAC (repeatable): written to
#                    RS_HOST_MAC in /etc/go2-realsense.env (added, never removed)
#   --hotplug        also start the service when the camera is plugged in later
#
# Writes only these files (no ROS, Unitree or network config is touched):
#   /opt/go2-realsense/go2_realsense_boot.sh
#   /etc/systemd/system/go2-realsense.service
#   /etc/go2-realsense.env                      (kept if it exists; only RS_HOST_MAC updated)
#   /etc/udev/rules.d/99-go2-realsense.rules    (only with --hotplug)
# ---------------------------------------------------------------------------
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT=/etc/systemd/system/go2-realsense.service
ENVF=/etc/go2-realsense.env
BIN=/opt/go2-realsense/go2_realsense_boot.sh
RULE=/etc/udev/rules.d/99-go2-realsense.rules

need_root() { [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }; }

MACS=() HOTPLUG=0 ACTION=install
while [ $# -gt 0 ]; do
    case "$1" in
        --host-mac) [ -n "${2:-}" ] || { echo "--host-mac needs a MAC" >&2; exit 2; }
                    m="$(tr '[:upper:]-' '[:lower:]:' <<< "$2")"
                    [[ "${m}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] || { echo "not a MAC: $2" >&2; exit 2; }
                    MACS+=("${m}"); shift ;;
        --hotplug)   HOTPLUG=1 ;;
        --uninstall) ACTION=uninstall ;;
        --status)    ACTION=status ;;
        *) sed -n '2,23p' "$0"; exit 2 ;;
    esac
    shift
done

case "${ACTION}" in
    status)
        systemctl is-enabled go2-realsense 2>/dev/null || true
        systemctl status go2-realsense --no-pager -l 2>/dev/null | head -20 || true
        grep -E '^RS_HOST_MAC=|^RS_DDS_PEERS=' "${ENVF}" 2>/dev/null || true
        [ -x "${BIN}" ] && echo "peers now: $(RS_ENV_FILE="${ENVF}" "${BIN}" --peers 2>/dev/null)"
        [ -f "${RULE}" ] && echo "hotplug rule: ${RULE}"
        exit 0 ;;
    uninstall)
        need_root
        systemctl disable --now go2-realsense 2>/dev/null || true
        rm -f "${UNIT}" "${BIN}" "${RULE}"
        rmdir /opt/go2-realsense 2>/dev/null || true
        systemctl daemon-reload
        udevadm control --reload-rules 2>/dev/null || true
        echo "removed (settings kept in ${ENVF}; delete it by hand if you want)"
        exit 0 ;;
esac

need_root
systemctl is-enabled docker >/dev/null 2>&1 || echo "WARNING: docker.service is not enabled at boot: sudo systemctl enable docker"

install -d /opt/go2-realsense
install -m 755 "${DIR}/go2_realsense_boot.sh" "${BIN}"
install -m 644 "${DIR}/go2-realsense.service" "${UNIT}"
if [ -f "${ENVF}" ]; then
    echo "keeping existing ${ENVF}"
    grep -q '^RS_HOST_MAC=' "${ENVF}" || echo 'RS_HOST_MAC=' >> "${ENVF}"
else
    install -m 644 "${DIR}/go2-realsense.env" "${ENVF}"
    echo "wrote ${ENVF}"
fi

if [ ${#MACS[@]} -gt 0 ]; then
    cur="$(sed -n 's/^RS_HOST_MAC=//p' "${ENVF}" | head -1 | tr -d '"')"
    # shellcheck disable=SC2086  # cur is a space-separated list on purpose
    new="$(printf '%s\n' ${cur} "${MACS[@]}" | sort -u | tr '\n' ' ' | sed 's/ $//')"
    sed -i "s/^RS_HOST_MAC=.*/RS_HOST_MAC=\"${new}\"/" "${ENVF}"
    echo "RS_HOST_MAC=\"${new}\""
fi
grep -qE '^RS_HOST_MAC="?[0-9a-f]' "${ENVF}" \
    || echo "NOTE: RS_HOST_MAC is empty: the camera finds no workstation until one is set (--host-mac) or a session pushes its IP."

if [ "${HOTPLUG}" = 1 ]; then
    cat > "${RULE}" <<'EOF'
# Start the RealSense camera service when a D435i is plugged in (go2-realsense).
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", TAG+="systemd", ENV{SYSTEMD_WANTS}+="go2-realsense.service"
EOF
    udevadm control --reload-rules
    echo "hotplug rule installed: plugging the camera in starts the service"
fi

systemctl daemon-reload
systemctl enable go2-realsense >/dev/null 2>&1
echo "enabled: the camera node starts at every boot when the D435i is plugged in."
if systemctl is-active --quiet go2-realsense; then
    systemctl restart go2-realsense && echo "restarted with the new settings"
fi
echo "start now:  sudo systemctl start go2-realsense      logs: journalctl -u go2-realsense -f"
