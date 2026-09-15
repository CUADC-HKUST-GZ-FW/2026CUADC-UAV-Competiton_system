# UAV reconnaissance geolocator

This read-only ROS 2 module joins timestamped vision targets with MAVROS RTK and
attitude telemetry. It projects the target pixel through the calibrated camera,
intersects the ray with a flat surveyed ground plane, and fuses repeated frames.

It never arms, changes mode, uploads a mission, or sends a position command.

Output:

```text
recon_results/current/
  status.json
  target_001/
    frame.jpg
    crop_128.jpg
    result.json
```

Before flight, set real camera intrinsics, camera-to-FCU lever arm, and surveyed
ground MSL altitude in `config/recon.yaml`.

Flight mode uses `tracking_mode=pixel_packets`. Non-empty three-point Pose
centres remain in one packet while detections are no more than 0.20 seconds
apart and pass the time-scaled pixel gate, capped at 200 pixels. The finalized
packet label is selected by valid-frame majority vote. Packets with
10 or fewer valid frames are rejected. A packet group also closes immediately
when any active packet reaches 300 valid frames. Once closed, packet coordinates
are resolved as follows:

- no more than 1.5 m apart: merge by valid frame count;
- more than 1.5 m but less than 10 m apart: keep the larger packet;
- at least 10 m apart: publish distinct targets.

Only the resulting `finalized` coordinates are forwarded to the flight bridge.
Static fixed-height calibration explicitly selects `legacy_geo_cluster`, so a
stationary target can continue updating without waiting for a packet to close.

On the Jetson:

```bash
/home/nx163/youth-vision-runtime/scripts/start_recon_pipeline.sh digit
/home/nx163/youth-vision-runtime/scripts/stop_recon_pipeline.sh
```

Use `image` instead of `digit` for the 12-class image classifier. The newest
session is available through `/home/nx163/youth-vision-runtime/recon_results/latest`.
