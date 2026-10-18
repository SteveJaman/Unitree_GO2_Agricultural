#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# go2_net.sh — shared network discovery for the Go2 Wi-Fi stack.
#
# Sourced (not executed) by wifi_nav.sh, wifi_mapping.sh and the tools.
#
# The router hands out addresses by DHCP, so nothing here is hardcoded:
#   * Host IP  : read from the Wi-Fi interface (GO2_IFACE, default enp0s3,
#                the VirtualBox bridged adapter).
#   * Robot IP : find_go2_ip() — cache → hostname → MAC scan → SSH probe.
#
# The D1 arm's wired interface (ARM_IFACE, default enp0s8) is never used:
# it is refused as GO2_IFACE, never scanned, and a robot route through it
# is an error. This keeps Go2 DDS traffic off the arm's segment.
#
# Tunables (env):
#   GO2_IFACE=enp0s3          Wi-Fi interface that reaches the Go2
#   ARM_IFACE=enp0s8          D1 arm interface — always ignored here
#   GO2_MAC_PREFIX=02:00:00   MAC prefix of the Go2 Jetson Wi-Fi
#   GO2_ROBOT_HOSTNAME=unitree
#   GO2_ROBOT_USER=unitree
#   GO2_IP_CACHE=~/.cache/go2_wifi_nav/robot_ip
# ---------------------------------------------------------------------------

GO2_IFACE="${GO2_IFACE:-enp0s3}"
ARM_IFACE="${ARM_IFACE:-enp0s8}"
GO2_MAC_PREFIX="${GO2_MAC_PREFIX:-02:00:00}"
GO2_ROBOT_HOSTNAME="${GO2_ROBOT_HOSTNAME:-unitree}"
GO2_ROBOT_USER="${GO2_ROBOT_USER:-unitree}"
GO2_IP_CACHE="${GO2_IP_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/go2_wifi_nav/robot_ip}"

# Host keys are pinned to an alias, not an IP: a DHCP change does not trigger
# "REMOTE HOST IDENTIFICATION HAS CHANGED", and a different device that picks
# up the old IP fails key verification instead of being trusted.
GO2_SSH_OPTS=(-o BatchMode=yes -o HostKeyAlias=go2-robot -o StrictHostKeyChecking=accept-new)

# Discovery logs go to stderr so $(find_go2_ip) captures only the IP.
go2_log() { echo "$@" >&2; }

go2_valid_ip() {
    local ip="$1" o
    [[ "$ip" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
    IFS=. read -r -a o <<< "$ip"
    for x in "${o[@]}"; do [ "$x" -le 255 ] || return 1; done
}

# First global IPv4 CIDR on an interface, e.g. 192.168.1.4/24
go2_iface_cidr() {
    ip -4 -o addr show dev "$1" scope global 2>/dev/null | awk '{print $4; exit}'
}

# Interface the kernel would use to reach an IP
go2_route_dev() {
    ip -4 route get "$1" 2>/dev/null \
        | awk '{for (i = 1; i < NF; i++) if ($i == "dev") { print $(i + 1); exit }}'
}

# True if IP is directly reachable (same L2 segment) via GO2_IFACE
go2_on_wifi_segment() {
    local route
    route="$(ip -4 route get "$1" 2>/dev/null || true)"
    [ "$(go2_route_dev "$1")" = "${GO2_IFACE}" ] && [[ "$route" != *" via "* ]]
}

go2_mac_of() {
    ip neigh show "$1" dev "${GO2_IFACE}" 2>/dev/null \
        | awk '{for (i = 1; i < NF; i++) if ($i == "lladdr") { print tolower($(i + 1)); exit }}'
}

go2_mac_matches() {
    local mac prefix
    mac="$(go2_mac_of "$1")"
    prefix="$(echo "${GO2_MAC_PREFIX}" | tr '[:upper:]' '[:lower:]')"
    [ -n "$mac" ] && [[ "$mac" == "$prefix"* ]]
}

# Passwordless SSH succeeds and the host key matches the pinned go2-robot key.
# Only meaningful once --setup-ssh has run (alias is in known_hosts).
go2_ssh_probe() {
    ssh-keygen -F go2-robot >/dev/null 2>&1 || return 1
    ssh -o StrictHostKeyChecking=yes -o ConnectTimeout=2 "${GO2_SSH_OPTS[@]}" \
        "${GO2_ROBOT_USER}@$1" true >/dev/null 2>&1
}

# Is this IP the Go2? (MAC prefix or verified SSH identity)
go2_is_go2() {
    local ip="$1"
    go2_valid_ip "$ip" || return 1
    go2_on_wifi_segment "$ip" || return 1
    ping -c1 -W1 -I "${GO2_IFACE}" "$ip" >/dev/null 2>&1 || true   # refresh ARP
    go2_mac_matches "$ip" || go2_ssh_probe "$ip"
}

# ---------------------------------------------------------------------------
# Host IP. Sets GO2_HOST_IP from GO2_IFACE. Returns 1 with a message on error.
# ---------------------------------------------------------------------------
go2_detect_host_ip() {
    local cidr dev src

    if [ "${GO2_IFACE}" = "${ARM_IFACE}" ]; then
        go2_log "[net] ERROR: GO2_IFACE=${GO2_IFACE} is the D1 arm interface. Refusing."
        return 1
    fi

    if ! ip link show dev "${GO2_IFACE}" >/dev/null 2>&1; then
        # Interface name differs on this VM: fall back to the default route,
        # but never onto the arm's interface.
        read -r dev src < <(ip -4 route get 8.8.8.8 2>/dev/null | awk '
            {for (i = 1; i < NF; i++) { if ($i == "dev") d = $(i + 1); if ($i == "src") s = $(i + 1) }}
            END {print d, s}')
        if [ -z "${dev}" ] || [ -z "${src}" ]; then
            go2_log "[net] ERROR: ${GO2_IFACE} not found and no default route."
            return 1
        fi
        if [ "${dev}" = "${ARM_IFACE}" ]; then
            go2_log "[net] ERROR: ${GO2_IFACE} not found and the default route is the arm interface (${ARM_IFACE})."
            go2_log "             Set GO2_IFACE=<wifi iface> or pass --iface."
            return 1
        fi
        go2_log "[net] WARN: ${GO2_IFACE} not found — using default-route interface ${dev}"
        GO2_IFACE="${dev}"
        GO2_HOST_IP="${src}"
        return 0
    fi

    cidr="$(go2_iface_cidr "${GO2_IFACE}")"
    if [ -z "${cidr}" ]; then
        go2_log "[net] ERROR: ${GO2_IFACE} has no IPv4 address."
        go2_log "             Check the VirtualBox bridged adapter and DHCP (sudo dhclient ${GO2_IFACE})."
        return 1
    fi
    GO2_HOST_IP="${cidr%/*}"
}

# True if IP is assigned to GO2_IFACE (used to validate overrides)
go2_ip_on_wifi_iface() {
    ip -4 -o addr show dev "${GO2_IFACE}" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -Fqx "$1"
}

# ---------------------------------------------------------------------------
# Robot IP discovery. Prints the IP on stdout; logs on stderr.
# ---------------------------------------------------------------------------
_go2_populate_neighbors() {
    local cidr="$1" base i
    if command -v nmap >/dev/null 2>&1; then
        nmap -sn -n -e "${GO2_IFACE}" "${cidr}" >/dev/null 2>&1 || true
        return 0
    fi
    # No nmap: parallel ping sweep of the host's /24
    base="${GO2_HOST_IP%.*}"
    for i in $(seq 1 254); do
        ping -c1 -W1 -I "${GO2_IFACE}" "${base}.${i}" >/dev/null 2>&1 &
    done
    wait
}

_go2_pick() {
    # $@ = candidate IPs; print the first, warn if several
    [ $# -gt 0 ] || return 1
    [ $# -gt 1 ] && go2_log "[net] WARN: several Go2 candidates: $* — using $1 (override with --robot-ip)"
    echo "$1"
}

find_go2_ip() {
    local ip h cidr prefix cands=() neigh

    # 1. Last known IP, if it still answers as the Go2
    if [ -r "${GO2_IP_CACHE}" ]; then
        ip="$(head -n1 "${GO2_IP_CACHE}" 2>/dev/null || true)"
        if go2_is_go2 "${ip}"; then
            go2_log "[net] Robot found at cached IP ${ip}"
            echo "${ip}"; return 0
        fi
    fi

    # 2. Hostname (mDNS / router DNS)
    for h in "${GO2_ROBOT_HOSTNAME}" "${GO2_ROBOT_HOSTNAME}.local"; do
        ip="$(getent ahostsv4 "$h" 2>/dev/null | awk '{print $1; exit}' || true)"
        [ -n "${ip}" ] && [ "${ip}" != "${GO2_HOST_IP}" ] || continue
        if go2_on_wifi_segment "${ip}" && ping -c1 -W1 -I "${GO2_IFACE}" "${ip}" >/dev/null 2>&1; then
            go2_log "[net] Robot found by hostname ${h} -> ${ip}"
            echo "${ip}"; return 0
        fi
    done

    cidr="$(go2_iface_cidr "${GO2_IFACE}")"
    if [ -z "${cidr}" ]; then
        go2_log "[net] ERROR: ${GO2_IFACE} has no address — cannot scan."
        return 1
    fi
    prefix="$(echo "${GO2_MAC_PREFIX}" | tr '[:upper:]' '[:lower:]')"

    # 3a. arp-scan (needs root) — scans only GO2_IFACE
    if command -v arp-scan >/dev/null 2>&1; then
        local sudo_cmd=()
        [ "$(id -u)" -eq 0 ] || sudo_cmd=(sudo -n)
        if [ ${#sudo_cmd[@]} -eq 0 ] || sudo -n true 2>/dev/null; then
            go2_log "[net] Scanning ${cidr} on ${GO2_IFACE} with arp-scan for MAC ${prefix}*..."
            mapfile -t cands < <("${sudo_cmd[@]}" arp-scan -I "${GO2_IFACE}" -q "${cidr}" 2>/dev/null \
                | awk -v p="${prefix}" '$1 ~ /^[0-9.]+$/ && index(tolower($2), p) == 1 {print $1}' \
                | sort -u -t. -k4,4n)
            if _go2_pick "${cands[@]}"; then return 0; fi
        fi
    fi

    # 3b. nmap / ping sweep, then read the kernel neighbor table for GO2_IFACE
    go2_log "[net] Scanning ${cidr} on ${GO2_IFACE} for MAC ${prefix}*..."
    _go2_populate_neighbors "${cidr}"
    mapfile -t cands < <(ip neigh show dev "${GO2_IFACE}" 2>/dev/null \
        | awk -v p="${prefix}" '{for (i = 1; i < NF; i++) if ($i == "lladdr" && index(tolower($(i + 1)), p) == 1) print $1}' \
        | sort -u -t. -k4,4n)
    if _go2_pick "${cands[@]}"; then return 0; fi

    # 4. SSH identity probe across live neighbors (only after --setup-ssh)
    if ssh-keygen -F go2-robot >/dev/null 2>&1; then
        go2_log "[net] No MAC match — probing neighbors over SSH for the pinned go2-robot key..."
        mapfile -t neigh < <(ip neigh show dev "${GO2_IFACE}" 2>/dev/null \
            | awk '$0 !~ /FAILED|INCOMPLETE/ {print $1}' | grep -vx "${GO2_HOST_IP}" || true)
        mapfile -t cands < <(
            for ip in "${neigh[@]}"; do
                ( go2_ssh_probe "${ip}" && echo "${ip}" ) &
            done
            wait
        )
        if _go2_pick "${cands[@]}"; then return 0; fi
    fi

    go2_log "[net] ERROR: could not find the Go2 on ${GO2_IFACE} (${cidr})."
    return 1
}

# ---------------------------------------------------------------------------
# D1 arm interface (only used by `wifi_nav.sh --arm`).
# The arm talks DDS domain 0 on ARM_IFACE; the Go2 talks domain 64 bound to
# the GO2_IFACE address. They stay isolated only if the two interfaces are
# different and on non-overlapping subnets. Sets ARM_HOST_CIDR.
# ---------------------------------------------------------------------------
go2_ip_to_int() {
    local IFS=. a b c d
    read -r a b c d <<< "$1"
    echo $(( (a << 24) | (b << 16) | (c << 8) | d ))
}

go2_cidrs_overlap() {
    local i1 i2 p1="${1#*/}" p2="${2#*/}" p m
    i1="$(go2_ip_to_int "${1%/*}")"
    i2="$(go2_ip_to_int "${2%/*}")"
    p=$(( p1 < p2 ? p1 : p2 ))
    m=$(( p == 0 ? 0 : (0xFFFFFFFF << (32 - p)) & 0xFFFFFFFF ))
    [ $(( i1 & m )) -eq $(( i2 & m )) ]
}

go2_check_arm_iface() {
    local cidr wcidr
    ARM_HOST_CIDR=""
    if [ "${ARM_IFACE}" = "${GO2_IFACE}" ]; then
        go2_log "[arm] ERROR: ARM_IFACE and GO2_IFACE are both ${ARM_IFACE}."
        return 1
    fi
    if ! ip link show dev "${ARM_IFACE}" >/dev/null 2>&1; then
        go2_log "[arm] ERROR: arm interface ${ARM_IFACE} not found (attach the arm's adapter to the VM)."
        return 1
    fi
    cidr="$(go2_iface_cidr "${ARM_IFACE}")"
    if [ -z "${cidr}" ]; then
        go2_log "[arm] ERROR: ${ARM_IFACE} has no IPv4 address. The arm is 192.168.123.100, so e.g.:"
        go2_log "             ./scripts/runtime/start_arm.sh --setup-net   (adds 192.168.123.51/24 to ${ARM_IFACE})"
        return 1
    fi
    wcidr="$(go2_iface_cidr "${GO2_IFACE}")"
    if [ -n "${wcidr}" ] && go2_cidrs_overlap "${cidr}" "${wcidr}"; then
        go2_log "[arm] ERROR: ${ARM_IFACE} (${cidr}) and ${GO2_IFACE} (${wcidr}) share a subnet."
        go2_log "             Arm and Go2 DDS traffic would mix. Give ${ARM_IFACE} a 192.168.123.x address."
        return 1
    fi
    ARM_HOST_CIDR="${cidr}"
    export ARM_IFACE ARM_HOST_CIDR
}

go2_save_robot_ip() {
    mkdir -p "$(dirname "${GO2_IP_CACHE}")" 2>/dev/null || return 0
    echo "$1" > "${GO2_IP_CACHE}" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# One-call resolver used by the entry scripts.
#   go2_resolve_network <host_override> <robot_override> <robot_required>
# Sets GO2_HOST_IP, GO2_ROBOT_IP, GO2_HOST_IP_SRC, GO2_ROBOT_IP_SRC.
# Overrides come from --host-ip / --robot-ip; env GO2_HOST_IP / GO2_ROBOT_IP
# are treated as hints and ignored if stale (a common DHCP failure mode).
# ---------------------------------------------------------------------------
go2_resolve_network() {
    local host_flag="$1" robot_flag="$2" robot_required="${3:-true}"
    local env_host="${GO2_HOST_IP:-}" env_robot="${GO2_ROBOT_IP:-}" found

    # ---- Host ----
    if [ -n "${host_flag}" ]; then
        go2_valid_ip "${host_flag}" || { go2_log "[net] ERROR: --host-ip '${host_flag}' is not an IPv4 address."; return 1; }
        GO2_HOST_IP="${host_flag}"
        GO2_HOST_IP_SRC="--host-ip"
        if ! go2_ip_on_wifi_iface "${GO2_HOST_IP}"; then
            go2_log "[net] WARN: ${GO2_HOST_IP} is not assigned to ${GO2_IFACE} — DDS may not reach the robot."
        fi
    else
        go2_detect_host_ip || return 1
        GO2_HOST_IP_SRC="auto-detected on ${GO2_IFACE}"
        if [ -n "${env_host}" ] && [ "${env_host}" != "${GO2_HOST_IP}" ]; then
            go2_log "[net] NOTE: ignoring stale env GO2_HOST_IP=${env_host} (${GO2_IFACE} is ${GO2_HOST_IP}); use --host-ip to force."
        fi
    fi

    # ---- Robot ----
    if [ -n "${robot_flag}" ]; then
        go2_valid_ip "${robot_flag}" || { go2_log "[net] ERROR: --robot-ip '${robot_flag}' is not an IPv4 address."; return 1; }
        GO2_ROBOT_IP="${robot_flag}"
        GO2_ROBOT_IP_SRC="--robot-ip"
    elif [ -n "${env_robot}" ] && go2_valid_ip "${env_robot}" && \
         ping -c1 -W1 -I "${GO2_IFACE}" "${env_robot}" >/dev/null 2>&1; then
        GO2_ROBOT_IP="${env_robot}"
        GO2_ROBOT_IP_SRC="env GO2_ROBOT_IP"
    else
        [ -n "${env_robot}" ] && go2_log "[net] NOTE: env GO2_ROBOT_IP=${env_robot} does not answer — discovering instead."
        if found="$(find_go2_ip)"; then
            GO2_ROBOT_IP="${found}"
            GO2_ROBOT_IP_SRC="discovered"
        elif [ "${robot_required}" = "true" ]; then
            go2_log "             Pass the robot's address explicitly:  --robot-ip <IP>"
            go2_log "             (check the router's DHCP lease list, or run 'hostname -I' on the Jetson)"
            return 1
        else
            GO2_ROBOT_IP=""
            GO2_ROBOT_IP_SRC="not found"
        fi
    fi

    # ---- Safeguard: the robot must be reached via Wi-Fi, never the arm NIC ----
    if [ -n "${GO2_ROBOT_IP}" ]; then
        local dev
        dev="$(go2_route_dev "${GO2_ROBOT_IP}")"
        if [ "${dev}" = "${ARM_IFACE}" ]; then
            go2_log "[net] ERROR: route to robot ${GO2_ROBOT_IP} goes via the arm interface ${ARM_IFACE}."
            go2_log "             Fix the VM routes so ${GO2_IFACE} carries Go2 traffic."
            return 1
        fi
        if [ -n "${dev}" ] && [ "${dev}" != "${GO2_IFACE}" ]; then
            go2_log "[net] WARN: route to robot ${GO2_ROBOT_IP} uses ${dev}, not ${GO2_IFACE}."
        fi
    fi

    export GO2_HOST_IP GO2_ROBOT_IP GO2_IFACE GO2_HOST_IP_SRC GO2_ROBOT_IP_SRC
}
