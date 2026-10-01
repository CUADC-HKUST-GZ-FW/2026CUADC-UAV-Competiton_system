# Jetson systemd deployment

This service starts the real-hardware ROS 2 bringup without requiring a desktop
login. Before installation, verify the Jetson username, workspace path, Ethernet
addresses, UDP ports, and the MAVROS launch filename.

The checked-in defaults assume:

- user: `jetson`
- workspace: `/home/nx163/uav_ros2_project`
- Jetson MAVLink receive port: `14550`
- FCU address: `192.168.10.1:14550`

The real launch starts with `dry_run_goto=true`, `control_mode=DRY_RUN`, and
`enable_real_payload_release=false`. Do not enable real control merely to test
systemd startup.

## Install

```bash
cd /home/nx163/uav_ros2_project
chmod 755 scripts/start_uav.sh

sudo install -o root -g root -m 0644 \
  deploy/systemd/uav-bringup.service \
  /etc/systemd/system/uav-bringup.service

sudo systemd-analyze verify /etc/systemd/system/uav-bringup.service
sudo systemctl daemon-reload
sudo systemctl enable --now uav-bringup.service
```

## Inspect and maintain

```bash
systemctl status uav-bringup.service
journalctl -fu uav-bringup.service
journalctl -b -u uav-bringup.service
sudo systemctl restart uav-bringup.service
sudo systemctl stop uav-bringup.service
sudo systemctl disable uav-bringup.service
```

`network-online.target` only confirms that Ubuntu considers its local network
configuration complete. It does not prove that the FCU is online or that a
MAVLink heartbeat has arrived. ROS nodes must remain safe while disconnected and
must handle reconnection internally.

If USB serial is added later, grant access with:

```bash
sudo usermod -aG dialout jetson
```

Then log out and back in, or reboot. Prefer `/dev/serial/by-id/...` over a
potentially unstable `/dev/ttyUSB0` path.
