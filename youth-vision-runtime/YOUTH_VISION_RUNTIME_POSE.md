# Youth YOLOv26 Three-Point Runtime

The active Jetson pipeline is:

```text
1440x1080 MVS frame
  -> YOLO26n Pose, 1536x1536 letterbox
  -> tip + base_left + base_right
  -> square whose lower edge is the two-point base
  -> square extends toward the tip and is warped to 128x128
  -> selected image or digit classifier
```

The pose output is `1x300x15`:

```text
x1 y1 x2 y2 score class
tip_x tip_y tip_conf
base_left_x base_left_y base_left_conf
base_right_x base_right_y base_right_conf
```

Run digit classification:

```bash
cd /home/hkustgz26/youth-vision-runtime
./scripts/run_youth_vision.sh digit
```

Run image classification:

```bash
cd /home/hkustgz26/youth-vision-runtime
./scripts/run_youth_vision.sh image
```

Live output:

```text
http://10.7.175.20:8000/
```

The page shows both the annotated camera frame and the exact 128x128
classifier input.
