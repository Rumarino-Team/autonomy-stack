
## Simulation

### Clone and go in repo
```sh
git clone --recursive https://github.com/Rumarino-Team/autonomy-stack.git
cd ./autonomy-stack
```

## Quick Start with Docker (Recommended for CI/CD)

```bash
# Build the Docker image
docker build -t rumarino-headless:latest .

# Run headless simulation test
docker run --rm \
  --name headless-test \
  rumarino-headless:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    ros2 launch bringup test_mission_executor_headless.launch.py \
      mission_name:=prequalify \
      controller_name:=stonefish_hydrus \
      env_file_name:=hydrus_env_headless.scn &
    LAUNCH_PID=\$! && \
    sleep 15 && \
    kill \$LAUNCH_PID 2>/dev/null || true
  "
```

## Local Development Setup
 System Dependencies

### Required Tools
- Python 3
- C++ compiler (GCC)
- Rust

### Fedora:
```sh
# Install essential build tools
sudo dnf install python3 python3-pip gcc gcc-c++ rust cargo

# Install ROS 2
sudo dnf copr enable tavie/ros2
sudo dnf install ros-jazzy-desktop
sudo dnf install ros-jazzy-vision-msgs
sudo dnf install freetype-devel
sudo dnf install SDL2-devel
sudo dnf install glm-devel
sudo dnf install eigen3-devel
sudo dnf install ogre-devel
sudo dnf install opencv-devel
sudo dnf install openssl-devel
sudo dnf install boost-devel
sudo dnf install libepoxy-devel
python3 -m pip install wheel
```
### Ubuntu:
```bash
# Install essential build tools
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential curl

# Install Clang/LLVM (required for Rust ROS 2 bindings)
sudo apt install -y libclang-dev llvm-dev clang

# Add ROS 2 repository (if not already added)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 and dependencies
sudo apt update
sudo apt install -y ros-jazzy-desktop \
    ros-jazzy-vision-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-rviz2 \
  ros-jazzy-usb-cam \
    libfreetype6-dev \
    libsdl2-dev \
    libglm-dev \
    libeigen3-dev \
  libogre-1.12-dev \
    libopencv-dev \
    libssl-dev \
    libboost-all-dev \
    libepoxy-dev \
    libtinyxml2-dev \
  ros-jazzy-pangolin \
    socat \
    pkg-config
```

### Install Stonefish Simulator
```sh
cd ./vendor/stonefish
mkdir build
cd build
cmake ..
make -j16 # (where X is the number of threads)
sudo make install
cd ../../../../../
```

## Computer Vision

### ZED Custom Wrapper

### Dependencies
  - [ZED-SDK 5.1](https://www.stereolabs.com/developers/release)
  - [Cuda 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive)

```sh
colcon build --packages-select zed_msg zed_custom_wrapper && source ./install/setup.bash && ros2 launch zed_custom_wrapper zed_custom.launch.py onnx_model_path:=./src/zed_custom_wrapper/yolov8n.onnx
```

### Jetson run using prebuilt Orb-Slam3 libraries

This is the recommended path on Jetson devices when ORB-SLAM3 cannot be compiled locally.

#### 1. Download the prebuilt bundle
```bash
cd /home/cesar/autonomy-stack
curl -L -o /tmp/orbslam-artifacts.zip \
  https://github.com/Rumarino-Team/autonomy-stack/releases/download/orb_slam_libs/artifacts.zip
```

#### 2. Install the bundle into the workspace (Jetson Only)
```bash
rm -rf /tmp/orbslam-artifacts-extract
mkdir -p /tmp/orbslam-artifacts-extract
unzip -o /tmp/orbslam-artifacts.zip -d /tmp/orbslam-artifacts-extract
ART=/tmp/orbslam-artifacts-extract/artifacts/orbslam-arm64/
sudo install -m 644 $ART/lib/* /usr/lib/ && sudo ldconfig
```

#### 3. Run the launch file
```bash
ros2 launch orb_slam3_ros2 orb_slam_sim_launch.py
```


Example using all optional arguments:

```bash
ros2 launch orb_slam3_ros2 orb_slam_sim_launch.py \
  use_viewer:=true \
  use_imu:=true \
  use_depth:=false \
  image_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  imu_topic:=/vectornav/imu \
  settings_file:=/home/cesar/autonomy-stack/src/orb_slam3_ros2/config/webcamera.yaml \
  use_usb_cam:=true \
  use_vectornav:=true
```

#### Notes
* The artifact bundle is built for Jetson-compatible Ubuntu 22.04 / ROS Humble ABI levels.

## Kalibr (Camera + IMU calibration)

Use the repository Kalibr wrapper in `tools/kalibr` to calibrate camera intrinsics and camera-IMU extrinsics.

### 1) Prepare tools
```sh
docker pull stereolabs/kalibr:latest
chmod +x tools/kalibr/run_kalibr.sh
chmod +x tools/kalibr/extract_orbslam_tbc.py
```

### 2) Record calibration bags
```sh
# Camera intrinsics bag
ros2 bag record /usb_cam/image_raw -o bags/cam_intrinsics
# Camera + IMU bag for extrinsics
ros2 bag record /usb_cam/image_raw /vectornav/imu -o bags/imucam
```

### 3) Calibrate camera intrinsics
```sh
bash tools/kalibr/run_kalibr.sh kalibr_calibrate_cameras \
  --bag /work/bags/cam_intrinsics.bag \
  --topics /usb_cam/image_raw \
  --models pinhole-radtan \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml
```

Output: `camchain-cam_intrinsics.yaml`

### 4) Prepare IMU noise config
Edit `tools/kalibr/config/imu.yaml` and fill real Allan-variance noise values.

### 5) Calibrate camera-IMU extrinsics
```sh
bash tools/kalibr/run_kalibr.sh kalibr_calibrate_imu_camera \
  --bag /work/bags/imucam.bag \
  --cam /work/camchain-cam_intrinsics.yaml \
  --imu /work/tools/kalibr/config/imu.yaml \
  --target /work/tools/kalibr/config/aprilgrid_6x6_80x30.yaml
```

Output: `camchain-imucam.yaml`

### 6) Get ORB-SLAM calibration matrix
```sh
python3 tools/kalibr/extract_orbslam_tbc.py camchain-imucam.yaml
```

Copy the printed matrix into both keys in `src/orb_slam3_ros2/config/webcamera.yaml`:
- `IMU.T_b_c1`
- `Tbc`

### Notes
- If Kalibr cannot read your ROS 2 bag directly, convert it to ROS1 bag format first.
- Ensure the topic names passed to Kalibr exactly match your recorded bag topics.




## Build using bridge_stonefish
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build \
    --packages-select interfaces bringup mission_executor bridge_stonefish Stonefish stonefish_ros2 detection_mocker joy sdl2_vendor

# Source the workspace
source install/setup.bash
```

## Simulate Missions using bridge_stonefish
```sh
# Note:
# - For `env_file_name:=pool_env.scn`, the launch script auto-selects the
#   matching pool scenario wrapper for the chosen AUV.
# - `auv_file_name` is optional and only needed as a manual override.

# proteus, prequalify mission
ros2 launch bringup stonefish.launch.py \
    mission_name:=prequalify \
    auv_name:=proteus \
    env_file_name:=proteus_env.scn \
    headless:=false

# hydrus, prequalify mission
ros2 launch bringup stonefish.launch.py \
    mission_name:=prequalify \
    auv_name:=hydrus \
    env_file_name:=hydrus_env.scn \
    headless:=false

# proteus, teleop mission
# if you don't have xterm, set TERMINAL to your terminal or install xterm.
# sudo apt install xterm
# sudo dnf install xterm
ros2 launch bringup stonefish.launch.py \
    mission_name:=teleop \
    auv_name:=proteus \
    env_file_name:=proteus_env.scn \
    headless:=false

# bluerov2, teleop mission
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  headless:=false

# optional manual override (advanced)
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  auv_file_name:=bluerov2.scn \
  headless:=false
```


## Build & Run proteus using bridge_hardware

### Build
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build \
  --packages-select interfaces bringup mission_executor bridge_hardware vectornav vectornav_msgs \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Run with real Arduino
```sh
# Source workspace after build
source install/setup.bash

# Launch full hardware stack
ros2 launch bringup hardware_proteus.launch.py \
  mission_name:=prequalify \
  arduino_port:=/dev/ttyACM0 \
  arduino_baud_rate:=115200

# Or run bridge node directly
ros2 run bridge_hardware bridge_proteus_node --ros-args \
  -p arduino_port:=/dev/ttyACM0 \
  -p arduino_baud_rate:=115200
```


### Arduino CLI (real board)

#### 1. Install AVR board support and Servo library dependency
```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
```
#### 2. Compile and upload Proteus firmware
```sh
arduino-cli compile --fqbn arduino:avr:uno arduino/sketches/Proteus
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/sketches/Proteus
```

### Mock Arduino test

#### 1. Create connected virtual serial ports
```sh
rm -f /tmp/ttyMOCK /tmp/ttyBRIDGE /tmp/socat_pair.log
socat -d -d \
  pty,raw,echo=0,link=/tmp/ttyMOCK \
  pty,raw,echo=0,link=/tmp/ttyBRIDGE \
  2>&1 | tee /tmp/socat_pair.log
```

#### 2. Start mock Arduino (new terminal)
```sh
python3 tools/mock_arduino.py /tmp/ttyMOCK 115200
```

#### 3. Start bridge_hardware with virtual bridge port (new terminal)
```sh
source install/setup.bash
ros2 run bridge_hardware bridge_proteus_node --ros-args \
  -p arduino_port:=/tmp/ttyBRIDGE \
  -p arduino_baud_rate:=115200
```

#### 4. Publish thruster commands (new terminal)
```sh
source install/setup.bash
ros2 topic pub -r 10 /bridge/thrusters std_msgs/msg/Float64MultiArray "{data: [0.45, -0.25, 0.15, -0.15, 0.05, -0.05]}"
```
