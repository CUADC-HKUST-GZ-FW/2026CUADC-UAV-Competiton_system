# UAV reconnaissance geolocator

This read-only ROS 2 module joins timestamped vision targets with MAVROS RTK and
attitude telemetry. It projects the target pixel through the calibrated camera,
intersects the ray with a flat surveyed ground plane, and fuses repeated passes.

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
ground MSL altitude in `config/recon.yaml`. A coordinate is confirmed only with
RTK Fixed and a bounded multi-frame 95 percent horizontal radius.

On the Jetson:

```bash
/home/nx163/youth-vision-runtime/scripts/start_recon_pipeline.sh digit
/home/nx163/youth-vision-runtime/scripts/stop_recon_pipeline.sh
```

Use `image` instead of `digit` for the 12-class image classifier. The newest
session is available through `/home/nx163/youth-vision-runtime/recon_results/latest`.
