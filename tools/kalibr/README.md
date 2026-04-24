# Kalibr Workflow (Docker on Ubuntu 24.04 + ROS 2 Jazzy)

This workspace uses Kalibr through Docker because native Kalibr is ROS1/catkin-based.

## Installed

Kalibr image installed locally:

- `stereolabs/kalibr:latest`

Wrapper script:

- `tools/kalibr/run_kalibr.sh`

## 0) Make wrapper executable

```bash
chmod +x tools/kalibr/run_kalibr.sh
chmod +x tools/kalibr/extract_orbslam_tbc.py
```

## 1) Record calibration data

### Camera intrinsics bag

Record only camera topic while moving AprilGrid through FOV:

```bash
ros2 bag record /camera/image_raw -o bags/cam_intrinsics
```

Convert to ROS1 bag if needed for Kalibr (Kalibr image expects ROS1 bag format).

### IMU + camera bag for extrinsics

Record synchronized camera + IMU while doing rich 6-DoF motion:

```bash
ros2 bag record /camera/image_raw /imu/data -o bags/imucam
```

Tips:

- Use 2-3 minutes of motion with rotations around all axes.
- Keep the calibration target visible often.
- Avoid motion blur.

## 2) Calibrate camera intrinsics

Use your AprilGrid config (example provided):

- `tools/kalibr/config/aprilgrid_6x6_80x30.yaml`

Run:

```bash
./tools/kalibr/run_kalibr.sh kalibr_calibrate_cameras \
  --bag /work/bags/cam_intrinsics.bag \
  --topics /camera/image_raw \
  --models pinhole-radtan \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml
```

Outputs include a `camchain-*.yaml` file.

## 3) Estimate IMU noise constants (Allan variance)

Kalibr needs an `imu.yaml` with real noise constants.

Use a long static IMU recording (at least 30 minutes, ideally 2+ hours), then compute:

- `accelerometer_noise_density`
- `accelerometer_random_walk`
- `gyroscope_noise_density`
- `gyroscope_random_walk`

Template:

- `tools/kalibr/config/imu.yaml.template`

Save your filled file as `tools/kalibr/config/imu.yaml`.

## 4) Calibrate IMU-camera extrinsics (T_b_c1 / Tbc)

Run Kalibr IMU-camera calibration with:

```bash
./tools/kalibr/run_kalibr.sh kalibr_calibrate_imu_camera \
  --bag /work/bags/imucam.bag \
  --cam /work/camchain-cam_intrinsics.yaml \
  --imu /work/tools/kalibr/config/imu.yaml \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml
```

Main output: `camchain-imucam-*.yaml`

## 5) Convert to ORB-SLAM matrix block

Extract `T_cam_imu` and print ORB-SLAM-ready matrix:

```bash
python3 tools/kalibr/extract_orbslam_tbc.py camchain-imucam.yaml
```

Then copy the printed matrix into both keys in your ORB-SLAM config:

- `IMU.T_b_c1`
- `Tbc`

In your file:

- `src/orb_slam3_ros2/config/webcamera.yaml`

## Notes

- If your ROS2 recording is not directly readable by Kalibr, convert your ROS2 bag to ROS1 bag format before running Kalibr.
- Ensure topic names in `imu.yaml` and command-line args exactly match your bag.

## VectorNav + Laptop Webcam Quickstart

This is a practical recipe for calibrating a VectorNav IMU with a laptop webcam.

1. Start the sensors

Typical commands (adjust to your setup):

```bash
ros2 launch vectornav vectornav.launch.py
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

2. Verify real topic names and rates

```bash
ros2 topic list | grep -E 'vectornav|imu|usb_cam|image_raw'
ros2 topic hz /vectornav/imu
ros2 topic hz /usb_cam/image_raw
```

If your IMU topic is different, use that exact topic in the IMU yaml.

3. Record calibration bags

```bash
ros2 bag record /usb_cam/image_raw -o bags/cam_intrinsics
ros2 bag record /usb_cam/image_raw /vectornav/imu -o bags/imucam
```

4. Use VectorNav IMU template

Start from:

- `tools/kalibr/config/imu_vectornav.yaml.template`

Copy it to `tools/kalibr/config/imu.yaml` and replace all four noise/random-walk constants with Allan-variance results from your own IMU.

5. Run calibrations (same commands as above)

```bash
bash tools/kalibr/run_kalibr.sh kalibr_calibrate_cameras \
  --bag /work/bags/cam_intrinsics.bag \
  --topics /usb_cam/image_raw \
  --models pinhole-radtan \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml

bash tools/kalibr/run_kalibr.sh kalibr_calibrate_imu_camera \
  --bag /work/bags/imucam.bag \
  --cam /work/camchain-cam_intrinsics.yaml \
  --imu /work/tools/kalibr/config/imu.yaml \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml
```

6. Update ORB-SLAM matrix block

```bash
python3 tools/kalibr/extract_orbslam_tbc.py camchain-imucam.yaml
```

Then paste into:

- `IMU.T_b_c1` in `src/orb_slam3_ros2/config/webcamera.yaml`
- `Tbc` in `src/orb_slam3_ros2/config/webcamera.yaml`
