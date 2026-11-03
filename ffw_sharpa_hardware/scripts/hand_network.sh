#!/usr/bin/env bash
# Runs on the robot host (not in a container; asks for sudo). Gives the Sharpa hands' USB Ethernet
# the static host address 192.168.10.240/24 with persistent NetworkManager profiles: no gateway,
# never the default route, no DNS. The robot's own eth0 / router network is not touched.
#
#   bash hand_network.sh                 # dry run: show what would change
#   bash hand_network.sh --apply         # apply (asks for the sudo password)
#   bash hand_network.sh --interface enxA [--interface enxB] ...   # choose adapters
#
# One adapter (hands behind a switch): profile "sharpa-hands" on that interface.
# Two or more adapters (one per hand): a software bridge "sharpa0" (profile "sharpa-hands", STP
# off) with the adapters as ports ("sharpa-hands-port-<if>"), so the hands share one segment and
# one address, as with a switch. Profiles previously bound to those adapters are saved under
# ~/.local/state/sharpa-hands/ and removed. Adapters are found as USB-attached wired interfaces
# that do not carry the default route; they should enumerate on USB 3 (5000M in `lsusb -t`).
set -euo pipefail

BACKUP_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/sharpa-hands"
PROFILE="sharpa-hands"
BRIDGE="sharpa0"
ADDRESS="192.168.10.240/24"
HANDS=("192.168.10.20" "192.168.10.10")  # right, left (factory addresses)
apply=0
interfaces=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --apply) apply=1 ;;
        --interface) interfaces+=("$2"); shift ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

command -v nmcli >/dev/null || { echo "NetworkManager (nmcli) is required" >&2; exit 1; }
default_if="$(ip route show default | awk '{print $5; exit}')"
if [[ ${#interfaces[@]} -eq 0 ]]; then
    for path in /sys/class/net/*; do
        name="${path##*/}"
        [[ -e "$path/device" && "$(cat "$path/type")" == 1 ]] || continue  # physical Ethernet
        [[ -d "$path/wireless" || -d "$path/phy80211" ]] && continue
        [[ "$name" == "$default_if" ]] && continue
        readlink -f "$path/device" | grep -q '/usb' || continue  # USB adapters only
        interfaces+=("$name")
    done
fi
[[ ${#interfaces[@]} -gt 0 ]] || { echo "No USB Ethernet adapter found; pass --interface" >&2; exit 1; }
for iface in "${interfaces[@]}"; do
    [[ -e "/sys/class/net/$iface" ]] || { echo "No interface $iface" >&2; exit 1; }
    [[ "$iface" != "$default_if" ]] || { echo "$iface carries the default route; refusing" >&2; exit 1; }
    carrier="$(cat "/sys/class/net/$iface/carrier" 2>/dev/null || echo 0)"
    speed="$(cat "/sys/class/net/$iface/speed" 2>/dev/null || echo "?")"
    driver="$(basename "$(readlink -f "/sys/class/net/$iface/device/driver")")"
    echo "Adapter: $iface (driver $driver, link $([[ $carrier == 1 ]] && echo "up $speed Mb/s" || echo down))"
done

ip_settings=(ipv4.method manual ipv4.addresses "$ADDRESS" ipv4.gateway "" ipv4.never-default yes
    ipv4.ignore-auto-dns yes ipv4.dns "" ipv6.method link-local connection.autoconnect yes)
field() { nmcli -g "$1" connection show "$2" 2>/dev/null || true; }
profiles_on() {  # every profile bound to an interface by name (active or not)
    nmcli -g NAME,UUID connection show | while IFS=: read -r name uuid; do
        [[ "$(field connection.interface-name "$uuid")" == "$1" ]] && echo "$uuid"
    done
    nmcli -g GENERAL.CONNECTION device show "$1" 2>/dev/null | grep -vx -- '--' | while read -r name; do
        [[ -n "$name" ]] && field connection.uuid "$name"
    done
}
ip_ok() {
    [[ "$(field ipv4.method "$1")" == "manual" && "$(field ipv4.addresses "$1")" == "$ADDRESS" &&
        "$(field ipv4.never-default "$1")" == "yes" ]]
}

declare -a plan=()
if [[ ${#interfaces[@]} -eq 1 ]]; then
    iface="${interfaces[0]}"
    if ip_ok "$PROFILE" && [[ "$(field connection.type "$PROFILE")" == "802-3-ethernet" &&
        "$(field connection.interface-name "$PROFILE")" == "$iface" ]]; then
        echo "Already configured: profile $PROFILE, $ADDRESS on $iface"
    else
        plan+=("save+delete profiles on $iface and an old $PROFILE"
            "nmcli connection add type ethernet con-name $PROFILE ifname $iface ${ip_settings[*]}"
            "nmcli connection up $PROFILE")
    fi
    target="$iface"
else
    configured=1
    ip_ok "$PROFILE" && [[ "$(field connection.type "$PROFILE")" == "bridge" &&
        "$(field connection.interface-name "$PROFILE")" == "$BRIDGE" ]] || configured=0
    for iface in "${interfaces[@]}"; do
        [[ "$(field connection.master "$PROFILE-port-$iface")" == "$BRIDGE" ]] || configured=0
    done
    if [[ $configured -eq 1 ]]; then
        echo "Already configured: bridge $BRIDGE ($PROFILE, $ADDRESS) with ports ${interfaces[*]}"
    else
        plan+=("save+delete profiles on ${interfaces[*]} and an old $PROFILE"
            "nmcli connection add type bridge con-name $PROFILE ifname $BRIDGE bridge.stp no ${ip_settings[*]}")
        for iface in "${interfaces[@]}"; do
            plan+=("nmcli connection add type ethernet con-name $PROFILE-port-$iface ifname $iface slave-type bridge master $BRIDGE")
        done
        plan+=("nmcli connection up $PROFILE")
    fi
    target="$BRIDGE"
fi

if [[ ${#plan[@]} -gt 0 ]]; then
    echo "Plan:"
    printf '  %s\n' "${plan[@]}"
    if [[ $apply -ne 1 ]]; then
        echo "Dry run. Rerun with --apply to change the network (sudo)."
        exit 0
    fi
    backup_dir="$BACKUP_DIR"
    mkdir -p "$backup_dir"
    stamp="$(date +%Y%m%d-%H%M%S)"
    old=()
    for iface in "${interfaces[@]}"; do
        while read -r uuid; do [[ -n "$uuid" ]] && old+=("$uuid"); done < <(profiles_on "$iface")
    done
    uuid="$(field connection.uuid "$PROFILE")"
    [[ -n "$uuid" ]] && old+=("$uuid")
    for uuid in $(printf '%s\n' "${old[@]}" | sort -u); do
        nmcli connection show "$uuid" > "$backup_dir/$stamp-$uuid.txt"
        echo "Saved profile $(field connection.id "$uuid") to $backup_dir/$stamp-$uuid.txt; removing it"
        sudo nmcli connection delete "$uuid" >/dev/null
    done
    if [[ ${#interfaces[@]} -eq 1 ]]; then
        sudo nmcli connection add type ethernet con-name "$PROFILE" ifname "${interfaces[0]}" "${ip_settings[@]}" >/dev/null
    else
        sudo nmcli connection add type bridge con-name "$PROFILE" ifname "$BRIDGE" bridge.stp no \
            "${ip_settings[@]}" >/dev/null
        for iface in "${interfaces[@]}"; do
            sudo nmcli connection add type ethernet con-name "$PROFILE-port-$iface" ifname "$iface" \
                slave-type bridge master "$BRIDGE" >/dev/null
        done
    fi
    sudo nmcli connection up "$PROFILE" >/dev/null
    if [[ ${#interfaces[@]} -gt 1 ]]; then
        for iface in "${interfaces[@]}"; do
            sudo nmcli connection up "$PROFILE-port-$iface" >/dev/null
        done
        echo "To undo: sudo nmcli connection delete $PROFILE ${interfaces[*]/#/$PROFILE-port-}"
    else
        echo "To undo: sudo nmcli connection delete $PROFILE"
    fi
fi

# Verify: our address, the default route unchanged, and which hands answer.
for _ in $(seq 30); do
    ip -4 -br addr show dev "$target" 2>/dev/null | grep -q "${ADDRESS%/*}" && break
    sleep 0.5
done
ip -4 -br addr show dev "$target"
if [[ "$(ip route show default | awk '{print $5; exit}')" != "$default_if" ]]; then
    echo "WARNING: the default route moved off $default_if" >&2
    exit 1
fi
echo "Default route still via $default_if"
sleep 2  # a new bridge port forwards after a short delay
for host in "${HANDS[@]}"; do
    if ping -c 2 -W 1 -I "$target" "$host" >/dev/null 2>&1; then
        echo "  $host answers"
    else
        echo "  $host does not answer (not connected or powered?)"
    fi
done
