#!/bin/bash
# nx164 相机口网络配置（对应《GigE 网口相机接入指南》§5.1）
# 需要 root：printf 'PASS\n' | sudo -S -p '' bash nx164_setup_camport.sh
set -x
IFACE=enP8p1s0
PROFILE="Wired connection 1"   # nx164 上 enP8p1s0 的默认 profile 名

# 1) 重命名成设备名，和文档保持一致
nmcli connection show "$PROFILE" >/dev/null 2>&1 && \
  nmcli connection modify "$PROFILE" connection.id "$IFACE"

# 2) 一次设全，不要拆成多条。
#    ⚠ 坑：ipv4.method=manual 在 ipv4.addresses 还是空的时候单独设置会被拒绝
#    （"this property cannot be empty for 'method=manual'"），该条 modify 整个不生效，
#    method 停在 auto。之后 NM 一直跑 DHCP，超时后把设备 deactivate ——
#    现象是"刚配好能用，过一会儿 IP 和 MTU 全没了、相机 devices=0"。
nmcli connection modify "$IFACE" \
  ipv4.addresses 192.168.10.1/24 \
  ipv4.method manual \
  ipv4.never-default yes \
  802-3-ethernet.mtu 9000 \
  connection.autoconnect yes

# 3) 生效
nmcli connection up "$IFACE"

set +x
echo "=== RESULT ==="
ip -br addr show "$IFACE"
ip -o link show "$IFACE"
echo -n "ipv4.method = "; nmcli -g ipv4.method connection show "$IFACE"
echo "=== 别忘了锁频，否则推理只有一半帧率 ==="
echo "sudo jetson_clocks"
