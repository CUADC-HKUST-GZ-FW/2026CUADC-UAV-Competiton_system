#!/usr/bin/env bash
set -euo pipefail

readonly SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

if [[ ${EUID} -ne 0 ]]; then
    echo "Run with sudo: sudo $0" >&2
    exit 1
fi

for source_file in \
    uav-clock-persist \
    uav-clock-persist.service \
    uav-clock-persist.timer \
    youth-vision-time-order.conf; do
    if [[ ! -f "${SOURCE_DIR}/${source_file}" ]]; then
        echo "Missing deployment file: ${SOURCE_DIR}/${source_file}" >&2
        exit 1
    fi
done

install -o root -g root -m 0755 \
    "${SOURCE_DIR}/uav-clock-persist" \
    /usr/local/sbin/uav-clock-persist
install -o root -g root -m 0644 \
    "${SOURCE_DIR}/uav-clock-persist.service" \
    "${SOURCE_DIR}/uav-clock-persist.timer" \
    /etc/systemd/system/

if systemctl cat youth-vision.service >/dev/null 2>&1; then
    install -d -o root -g root -m 0755 \
        /etc/systemd/system/youth-vision.service.d
    install -o root -g root -m 0644 \
        "${SOURCE_DIR}/youth-vision-time-order.conf" \
        /etc/systemd/system/youth-vision.service.d/time-order.conf
fi

systemctl daemon-reload
systemctl enable --now uav-clock-persist.timer
systemctl start uav-clock-persist.service

echo "[OK] offline clock fallback installed"
systemctl show uav-clock-persist.timer \
    -p UnitFileState -p ActiveState -p SubState -p NextElapseUSecRealtime
stat -c '[CLOCK] %y %n' /var/lib/systemd/timesync/clock
