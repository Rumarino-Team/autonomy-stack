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
- C++ compiler (GCC) + CMake
- Rust ([rustup](https://rustup.rs))
- Clang/LLVM (for Rust ROS 2 bindings / r2r)

Install a **minimal** ROS 2 Jazzy (`ros-base`), not `desktop`. Optional tools (joy, usb_cam, rviz) are listed under the profiles that need them.

### Fedora:
```sh
# Build tools + Rust
sudo dnf install python3 python3-pip gcc gcc-c++ cmake pkgconf-pkg-config rust cargo

# ROS 2 (minimal) + sim msgs
sudo dnf copr enable tavie/ros2
sudo dnf install ros-jazzy-ros-base ros-jazzy-vision-msgs \
  ros-jazzy-image-transport ros-jazzy-pcl-conversions ros-jazzy-visualization-msgs \
  python3-colcon-common-extensions

# Stonefish + detection_mocker system libs (include SDL2 for Stonefish)
sudo dnf install freetype-devel glm-devel eigen3-devel tinyxml2-devel \
  mesa-libGL-devel libclang-devel clang SDL2-devel

python3 -m pip install wheel
```

### Ubuntu:
```bash
sudo apt update
sudo apt install -y python3 python3-pip build-essential cmake pkg-config curl \
  libclang-dev llvm-dev clang \
  python3-colcon-common-extensions

# Add ROS 2 repository (if not already added)
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# System dependencies
sudo apt update
sudo apt install -y ros-jazzy-ros-base \
  ros-jazzy-vision-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-pcl-conversions \
  ros-jazzy-visualization-msgs \
  libfreetype6-dev \
  libglm-dev \
  libeigen3-dev \
  libtinyxml2-dev \
  libgl1-mesa-dev

# Optional: system SDL2 (Stonefish uses libsdl2-dev on Linux)
sudo apt install -y libsdl2-dev

# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
```

### Optional profiles (install only if you need them)

```bash
# Teleop joystick (stonefish.launch.py use_joy:=true)
sudo apt install -y ros-jazzy-joy          # Fedora: ros-jazzy-joy

# Hardware Proteus camera / mock serial
sudo apt install -y ros-jazzy-usb-cam socat

# GUI visualization (not required for sim/CI)
sudo apt install -y ros-jazzy-rviz2
```

### Build
```sh
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Stonefish (`vendor/stonefish`) is a colcon cmake package and installs into `install/` with everything else. `stonefish_ros2` declares a build dependency on it, so colcon builds them in order.

Optional stacks (ZED vision) are ignored via `COLCON_IGNORE` under `src/zed_custom_wrapper/`.

## Computer Vision

### ZED Custom Wrapper

`zed_custom_wrapper` is ignored by default (`COLCON_IGNORE`) so a normal sim build stays lean. Remove that file before building vision.

### Dependencies
  - [ZED-SDK 5.1](https://www.stereolabs.com/developers/release)
  - [Cuda 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive)

```sh
rm -f src/zed_custom_wrapper/COLCON_IGNORE
# also init vendor/zed-ros-interfaces if you need zed_msgs
colcon build --packages-select zed_msgs zed_custom_wrapper && source ./install/setup.bash && \
  ros2 launch zed_custom_wrapper zed_custom.launch.py onnx_model_path:=./src/zed_custom_wrapper/yolov8n.onnx
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

# proteus, teleop mission (needs ros-jazzy-joy)
# if you don't have xterm, set TERMINAL to your terminal or install xterm.
# sudo apt install xterm
# sudo dnf install xterm
ros2 launch bringup stonefish.launch.py \
    mission_name:=teleop \
    auv_name:=proteus \
    env_file_name:=proteus_env.scn \
    headless:=false \
    use_joy:=true

# bluerov2, teleop mission
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  headless:=false \
  use_joy:=true

# optional manual override (advanced)
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  auv_file_name:=bluerov2.scn \
  headless:=false \
  use_joy:=true

# bluerov2 direct actuator sanity test
# Use stonefish_only so mission_executor does not overwrite the direct command.
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  headless:=false \
  stonefish_only:=true

# In another terminal, publish 8 normalized thruster values.
source install/setup.bash

# Equal horizontal commands cancel on this angled layout; this pattern drives body +X.
ros2 topic pub -r 10 /bridge/thrusters std_msgs/msg/Float64MultiArray \
  "{data: [-0.6, -0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0]}"

# Watch Stonefish's actuator feedback.
ros2 topic echo /bridge/thruster_state
```


## Build & Run proteus using bridge_hardware

### Extra deps
```sh
sudo apt install -y ros-jazzy-usb-cam socat   # usb_cam optional; socat for mock Arduino
```

### Build
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build \
  --packages-select interfaces bringup mission_executor bridge_hardware \
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
