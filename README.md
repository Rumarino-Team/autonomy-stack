# autonomy-stack

ROS 2 simulation and control stack for Rumarino AUVs (Stonefish + `mission_executor`).

**Supported platform:** Ubuntu (22.04 on Jetson, 24.04 on desktop — use the ROS 2 release that matches your Ubuntu version). This repo targets **ROS 2 Jazzy** on **Ubuntu 24.04**.

---

## 1. Clone

```sh
git clone --recursive https://github.com/Rumarino-Team/autonomy-stack.git
cd autonomy-stack
```

If you already cloned without submodules:

```sh
git submodule update --init --recursive
```

---

## 2. Install dependencies (Ubuntu)

Run from any directory. You only need to do this once per machine.

### 2a. ROS 2 repository (skip if ROS Jazzy is already installed)

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

### 2b. Build tools, ROS 2, and simulation libraries

```bash
sudo apt update
sudo apt install -y \
  python3 python3-pip build-essential cmake pkg-config curl \
  libclang-dev llvm-dev clang \
  python3-colcon-common-extensions \
  ros-jazzy-ros-base \
  ros-jazzy-vision-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-pcl-conversions \
  ros-jazzy-visualization-msgs \
  libfreetype6-dev \
  libglm-dev \
  libeigen3-dev \
  libtinyxml2-dev \
  libgl1-mesa-dev \
  libsdl2-dev
```

### 2c. Rust (required for `mission_executor`)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
```

### Optional (only if you need them)

```bash
# Teleop with a gamepad (pass use_joy:=true at launch)
sudo apt install -y ros-jazzy-joy xterm

# Hardware Proteus USB camera / mock serial
sudo apt install -y ros-jazzy-usb-cam socat
```

---

## 3. Build

From the **repo root** (`autonomy-stack/`):

```sh
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Stonefish is built by colcon as part of this command (via `vendor/stonefish`) and installs into `install/` with the rest of the workspace.

Re-build after code changes:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. Run simulation

Always launch from the **repo root** so config paths resolve correctly.

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Hydrus — prequalify mission (GUI)
ros2 launch bringup stonefish.launch.py \
  mission_name:=prequalify \
  auv_name:=hydrus \
  env_file_name:=hydrus_env.scn \
  headless:=false
```

Headless (no GPU window):

```sh
ros2 launch bringup stonefish.launch.py \
  mission_name:=prequalify \
  auv_name:=hydrus \
  env_file_name:=hydrus_env.scn \
  headless:=true
```

Proteus prequalify:

```sh
ros2 launch bringup stonefish.launch.py \
  mission_name:=prequalify \
  auv_name:=proteus \
  env_file_name:=proteus_env.scn \
  headless:=false
```

Teleop (requires `ros-jazzy-joy`; install in optional step above):

```sh
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=proteus \
  env_file_name:=proteus_env.scn \
  headless:=false \
  use_joy:=true
```

---

## More simulation examples

```sh
# pool scenario — env file is auto-selected per AUV
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  headless:=false \
  use_joy:=true

# direct thruster test (mission_executor disabled)
ros2 launch bringup stonefish.launch.py \
  mission_name:=teleop \
  auv_name:=bluerov2 \
  env_file_name:=pool_env.scn \
  headless:=false \
  stonefish_only:=true
```

In another terminal:

```sh
source install/setup.bash
ros2 topic pub -r 10 /bridge/thrusters std_msgs/msg/Float64MultiArray \
  "{data: [-0.6, -0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0]}"
ros2 topic echo /bridge/thruster_state
```

---

## Docker (CI / headless test)

```bash
docker build -t rumarino-headless:latest .

docker run --rm --name headless-test rumarino-headless:latest
```

---

## Hardware (Proteus + Arduino)

Extra dependency:

```sh
sudo apt install -y socat   # mock serial testing
```

Build:

```sh
source /opt/ros/jazzy/setup.bash
colcon build \
  --packages-select interfaces bringup mission_executor bridge_hardware \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run:

```sh
ros2 launch bringup hardware_proteus.launch.py \
  mission_name:=prequalify \
  arduino_port:=/dev/ttyACM0 \
  arduino_baud_rate:=115200
```

### Arduino firmware

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
arduino-cli compile --fqbn arduino:avr:uno arduino/sketches/Proteus
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/sketches/Proteus
```

### Mock Arduino (virtual serial)

Terminal 1:

```sh
socat -d -d pty,raw,echo=0,link=/tmp/ttyMOCK pty,raw,echo=0,link=/tmp/ttyBRIDGE
```

Terminal 2:

```sh
python3 tools/mock_arduino.py /tmp/ttyMOCK 115200
```

Terminal 3:

```sh
source install/setup.bash
ros2 run bridge_hardware bridge_proteus_node --ros-args \
  -p arduino_port:=/tmp/ttyBRIDGE \
  -p arduino_baud_rate:=115200
```

---

## Computer vision (ZED, optional)

`zed_custom_wrapper` is ignored by default (`src/zed_custom_wrapper/COLCON_IGNORE`).

Requires [ZED SDK 5.1](https://www.stereolabs.com/developers/release) and [CUDA 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive).

```sh
rm -f src/zed_custom_wrapper/COLCON_IGNORE
git submodule update --init vendor/zed-ros-interfaces
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zed_msgs zed_custom_wrapper
source install/setup.bash
ros2 launch zed_custom_wrapper zed_custom.launch.py \
  onnx_model_path:=./src/zed_custom_wrapper/yolov8n.onnx
```
