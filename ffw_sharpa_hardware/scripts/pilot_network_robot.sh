#!/usr/bin/env bash
# Temporary forwarding/NAT restricted to this PC and the two Sharpa hands.
set -euo pipefail
[[ "$EUID" == 0 ]] || { echo 'root required'; exit 1; }
action="${1:-}"
peer="${2:-}"
[[ "$action" == on || "$action" == off ]] || exit 2
/usr/bin/python3 -c 'import ipaddress,sys; p=ipaddress.ip_address(sys.argv[1]); assert p in ipaddress.ip_network("192.168.6.0/24") and p not in (ipaddress.ip_address("192.168.6.0"),ipaddress.ip_address("192.168.6.255"))' "$peer"
if [[ "$action" == on && "$(sysctl -n net.ipv4.ip_forward)" != 1 ]]; then
    echo 'IP forwarding is disabled; no changes made'; exit 1
fi
rule() {
    local table="$1" chain="$2"; shift 2
    if [[ "$action" == on ]]; then
        iptables -t "$table" -C "$chain" "$@" 2>/dev/null || iptables -t "$table" -I "$chain" 1 "$@"
    else
        if iptables -t "$table" -C "$chain" "$@" 2>/dev/null; then iptables -t "$table" -D "$chain" "$@"; fi
    fi
}
for hand in 192.168.10.10 192.168.10.20; do
    rule filter FORWARD -s "$peer/32" -d "$hand/32" -m comment --comment ai-worker-sharpa-pilot -j ACCEPT
    rule filter FORWARD -s "$hand/32" -d "$peer/32" -m conntrack --ctstate ESTABLISHED,RELATED -m comment --comment ai-worker-sharpa-pilot -j ACCEPT
    rule nat POSTROUTING -s "$peer/32" -d "$hand/32" -m comment --comment ai-worker-sharpa-pilot -j SNAT --to-source 192.168.10.240
done
echo "Pilot routing $action for $peer"
