#!/usr/bin/env bash
set -euo pipefail

USER_NAME="${USER_NAME:-nx163}"
HOST_NAME="$(hostname -s)"
MDNS_NAME="${HOST_NAME}.local"

echo "Network access information"
echo "=========================="
echo "hostname: $HOST_NAME"
echo "mDNS:     $MDNS_NAME"

default_iface="$(ip -4 route show default 2>/dev/null | awk 'NR == 1 {print $5}')"
if [[ -n "$default_iface" ]]; then
  default_ip="$(ip -o -4 addr show up dev "$default_iface" 2>/dev/null \
    | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
  [[ -n "$default_ip" ]] && echo "primary:  $default_ip ($default_iface)"
fi

if command -v nmcli >/dev/null 2>&1; then
  wifi_ssid="$(nmcli -t -f TYPE,STATE,CONNECTION device status 2>/dev/null \
    | awk -F: '$1 == "wifi" && $2 == "connected" {print $3; exit}')"
  [[ -n "$wifi_ssid" ]] && echo "WiFi SSID: $wifi_ssid"
fi

echo
echo "IPv4 addresses:"
found=0
while read -r iface address; do
  [[ "$iface" == "lo" ]] && continue
  state="$(cat "/sys/class/net/$iface/operstate" 2>/dev/null || echo unknown)"
  [[ "$state" == "down" ]] && continue
  found=1
  printf '  %-12s %-18s state=%s\n' "$iface" "$address" "$state"
done < <(ip -o -4 addr show up | awk '{split($4, a, "/"); print $2, a[1]}')
[[ "$found" -eq 0 ]] && echo "  none"

echo
echo "SSH commands:"
while read -r iface address; do
  [[ "$iface" == "lo" ]] && continue
  state="$(cat "/sys/class/net/$iface/operstate" 2>/dev/null || echo unknown)"
  [[ "$state" == "down" ]] && continue
  case "$address" in
    169.254.*) continue ;;
  esac
  printf '  %-12s ssh %s@%s\n' "$iface" "$USER_NAME" "$address"
done < <(ip -o -4 addr show up | awk '{split($4, a, "/"); print $2, a[1]}')
echo "  mDNS         ssh ${USER_NAME}@${MDNS_NAME}"

echo
echo "Notes:"
echo "  USB Device Mode normally uses: ssh ${USER_NAME}@192.168.55.1"
echo "  mDNS works only when the client network allows multicast discovery."
