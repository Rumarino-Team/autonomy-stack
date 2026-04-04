
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
    libfreetype6-dev \
    libsdl2-dev \
    libglm-dev \
    libeigen3-dev \
    libogre-1.9-dev \
    libopencv-dev \
    libssl-dev \
    libboost-all-dev \
    libepoxy-dev \
    libtinyxml2-dev \
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

### Building Orb Slam
```bash
cd vendor
wget https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/refs/heads/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xf ORBvoc.txt.tar.gz
git clone https://github.com/Cruiz102/ORB_SLAM3.git
#build everything with a single command
cd ORB_SLAM3
chmod +x build.sh
sudo ./build.sh
```




## Build using bridge_stonefish
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build \
    --packages-select interfaces bringup mission_executor bridge_stonefish Stonefish stonefish_ros2 detection_mocker

# Source the workspace
source install/setup.bash
```

## Simulate Missions using bridge_stonefish
```sh
# Note:
# - `auv_name` is now the single source of truth for AUV selection.
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
# if you don't have xterm
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

## Test using bridge_stonefish
```sh
# TODO
```

## Build & Run proteus using bridge_hardware
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages (TODO: remember about detections stuff)
colcon build \
    --packages-select interfaces bringup mission_executor bridge_hardware \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Source the workspace
source install/setup.bash

# Run
ros2 launch bringup hardware_proteus.launch.py \
    mission_name:=prequalify \
    arduino_port:=/dev/ttyACM0 arduino_baud_rate:=115200 \
    vn100_port:=/dev/ttyUSB0 vn100_baud_rate:=115200
```
