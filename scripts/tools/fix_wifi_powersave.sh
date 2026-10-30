#!/usr/bin/env bash
# fix_wifi_powersave.sh — stop the Wi-Fi dongles (laptop AND robot) from
# dozing off mid-run.
#
# Both ends of the Go2 link use Realtek USB dongles with Wi-Fi power saving on:
#   * workstation: RTL8822BU (rtw88_8822bu). Its firmware often fails to wake
#     up ("rtw_8822bu ...: firmware failed to leave lps state" in the kernel
#     log); on 2026-10-30, 30+ of those in 12 minutes, two full disconnects,
#     and a new DHCP lease (this computer's IP changed) each time;
#   * robot: RTL8821AU (out-of-tree 8821au) with rtw_power_mgnt=2 (maximum
#     power saving) and rtw_ips_mode=1, NetworkManager wifi.powersave = 3.
# A dozing receiver delays what is sent to it: SSH, DDS (LiDAR, cmd_vel) and
# the motion bridge stall for seconds, and wifi_3d_nav.sh runs ended ~15 s in
# (the link went silent for 70 s; RViz closed with the rest of the session).
#
# This turns power saving off for good (needs sudo, once per machine):
#   1. NetworkManager: wifi.powersave = 2 (off) for every Wi-Fi connection;
#   2. the driver: rtw88 disable_lps_deep=y, or for the 8821au / 8812au /
#      88x2bu / 8814au family rtw_power_mgnt=0 rtw_ips_mode=0;
#   3. applies it: here, reloads the driver and NetworkManager (Wi-Fi drops for
#      a few seconds). Over SSH (the robot) nothing is reloaded, that would cut
#      the connection: power saving goes off live with `iw`, the driver
#      setting applies at the next reboot.
#
# Usage:
#   ./scripts/tools/fix_wifi_powersave.sh            # this computer (asks for sudo)
#   ./scripts/tools/fix_wifi_powersave.sh --robot    # the Go2 (copies itself there;
#                                                    #   you type the robot's sudo password)
#   ./scripts/tools/fix_wifi_powersave.sh --check    # report only, no sudo
#   ./scripts/tools/fix_wifi_powersave.sh --undo     # remove the files written here
set -euo pipefail

NM_CONF=/etc/NetworkManager/conf.d/99-go2-wifi-powersave-off.conf
MOD_CONF=/etc/modprobe.d/go2-wifi-no-powersave.conf
RTL_FAMILY="8821au 8812au 88x2bu 8814au 8821cu"

wifi_ifaces() { for d in /sys/class/net/*/wireless; do [ -d "$d" ] && basename "$(dirname "$d")"; done; }
loaded() { [ -d "/sys/module/$1" ]; }

state() {
    local nm ifc m
    nm="$(grep -rhs '^wifi.powersave' /etc/NetworkManager/conf.d/ /etc/NetworkManager/NetworkManager.conf | tail -1 | tr -d ' ')"
    echo "  NetworkManager ${nm:-wifi.powersave=(default)}  (2 = off, 3 = on)"
    for ifc in $(wifi_ifaces); do
        if command -v iw >/dev/null; then
            echo "  $ifc: $(iw dev "$ifc" get power_save 2>/dev/null || echo 'power save: ?')"
        fi
    done
    if loaded rtw88_core; then
        echo "  rtw88_core disable_lps_deep=$(cat /sys/module/rtw88_core/parameters/disable_lps_deep)  (Y = off)"
        echo "  'failed to leave lps state' since boot: $(journalctl -k -b --no-pager 2>/dev/null | grep -c 'failed to leave lps state' || true)"
    fi
    for m in $RTL_FAMILY; do
        loaded "$m" || continue
        echo "  $m rtw_power_mgnt=$(cat /sys/module/$m/parameters/rtw_power_mgnt 2>/dev/null || echo ?)" \
             "rtw_ips_mode=$(cat /sys/module/$m/parameters/rtw_ips_mode 2>/dev/null || echo ?)  (0 = off)"
    done
}

case "${1:-}" in
    --check)
        echo "Wi-Fi power saving on $(hostname):"
        state
        exit 0 ;;
    --undo)
        sudo rm -f "$NM_CONF" "$MOD_CONF"
        echo "Removed $NM_CONF and $MOD_CONF; reboot to apply."
        exit 0 ;;
    --robot)
        here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        # shellcheck source=../runtime/go2_net.sh
        source "$here/../runtime/go2_net.sh"
        ip="${GO2_ROBOT_IP:-$(go2_known_robot_ip 2>/dev/null || true)}"
        [ -n "$ip" ] || { echo "Robot IP unknown: GO2_ROBOT_IP=... $0 --robot" >&2; exit 1; }
        user="${GO2_ROBOT_USER:-unitree}"
        echo "Fixing Wi-Fi power saving on the robot ${user}@${ip} (type its sudo password when asked)..."
        scp -q "${GO2_SSH_OPTS[@]}" "${BASH_SOURCE[0]}" "${user}@${ip}:/tmp/fix_wifi_powersave.sh"
        exec ssh -t "${GO2_SSH_OPTS[@]}" "${user}@${ip}" "bash /tmp/fix_wifi_powersave.sh" ;;
    "") ;;
    *) echo "usage: $0 [--check|--robot|--undo]" >&2; exit 2 ;;
esac

remote="${SSH_CONNECTION:+1}"
echo "Before ($(hostname)):"
state
echo
echo "Writing $NM_CONF (sudo)..."
printf '# fix_wifi_powersave.sh: power saving stalls the Go2 Wi-Fi link\n[connection]\nwifi.powersave = 2\n' |
    sudo tee "$NM_CONF" >/dev/null

opts=""
loaded rtw88_core && opts+="options rtw88_core disable_lps_deep=y\n"
for m in $RTL_FAMILY; do
    loaded "$m" && opts+="options $m rtw_power_mgnt=0 rtw_ips_mode=0\n"
done
if [ -n "$opts" ]; then
    echo "Writing $MOD_CONF (sudo)..."
    printf "# fix_wifi_powersave.sh: Realtek USB dongle power saving stalls the link\n${opts}" |
        sudo tee "$MOD_CONF" >/dev/null
fi

if [ -n "$remote" ]; then
    # Over SSH: reloading Wi-Fi would cut this very connection.
    if command -v iw >/dev/null; then
        for ifc in $(wifi_ifaces); do sudo iw dev "$ifc" set power_save off 2>/dev/null || true; done
    fi
    echo
    echo "After (live; the driver setting applies at the next reboot):"
    state
    echo
    echo "Done. Reboot the robot once when convenient: sudo reboot"
    exit 0
fi

if loaded rtw88_core; then
    echo "Reloading the rtw88 driver (Wi-Fi drops for a few seconds)..."
    mods="$(lsmod | awk '/^rtw88/ {print $1}')"
    mapfile -t chips < <(echo "$mods" | grep -v -e '^rtw88_core$' -e '^rtw88_usb$' -e '^rtw88_8822b$')
    sudo modprobe -r "${chips[@]}" rtw88_usb rtw88_8822b rtw88_core 2>/dev/null || true
    for m in $mods; do sudo modprobe "$m" 2>/dev/null || true; done
fi
for m in $RTL_FAMILY; do
    if loaded "$m"; then
        echo "Reloading $m (Wi-Fi drops for a few seconds)..."
        sudo modprobe -r "$m" && sudo modprobe "$m" || true
    fi
done
sudo systemctl restart NetworkManager
echo "Waiting for Wi-Fi to reconnect..."
for _ in $(seq 1 30); do
    nmcli -t -f DEVICE,STATE device 2>/dev/null | grep -q ':connected$' && break
    sleep 1
done
echo
echo "After:"
state
echo
echo "Done. If the kernel still logs 'failed to leave lps state', reboot once."
