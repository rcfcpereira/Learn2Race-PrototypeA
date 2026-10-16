# Learn2Race Camera Calibration Package

This package contains stereo calibration launch files for the two Raspberry Pi
Camera v2 modules.

## Algorithm

The calibration workflow uses the ROS `camera_calibration` GUI:

1. Start left and right raw image streams under `/stereo/left` and `/stereo/right`.
2. Disable rectification/disparity while calibrating, because calibration needs
   the raw camera images.
3. Show a checkerboard to both cameras at many distances and angles.
4. Let the GUI estimate each camera intrinsic matrix and the stereo transform.
5. Save the generated calibration files and copy them into the camera config
   location used by the final camera launch.

## Run

```bash
ros2 launch learn2race_camera_calibration stereo_camera_calibration.launch.py \
  checkerboard_size:=8x6 checkerboard_square_m:=0.025
```

## Required Packages

- `camera_ros`
- `camera_calibration`
- `image_proc`
- `stereo_image_proc`
- `launch`
- `launch_ros`
- `learn2race_bringup`

## References and more details

- [R34] Zhang, camera calibration; [R30] OpenCV; [R17] camera_ros; [R66] IMX219.
- Full entries: [`docs/references.md`](../../../docs/references.md).
