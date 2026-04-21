
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
### Jetson run using prebuilt artifacts

This is the recommended path on Jetson devices when ORB-SLAM3 cannot be compiled locally.

#### 1. Download the prebuilt bundle
```bash
cd /home/cesar/autonomy-stack
curl -L -o /tmp/orbslam-artifacts.zip \
  https://github.com/Rumarino-Team/autonomy-stack/releases/download/orb_slam_libs/artifacts.zip
```

#### 2. Install the bundle into the workspace
```bash
rm -rf /tmp/orbslam-artifacts-extract
mkdir -p /tmp/orbslam-artifacts-extract
unzip -o /tmp/orbslam-artifacts.zip -d /tmp/orbslam-artifacts-extract

ART=/tmp/orbslam-artifacts-extract/artifacts/orbslam-arm64

mkdir -p \
  vendor/ORB_SLAM3/lib \
  vendor/ORB_SLAM3/Thirdparty/DBoW2/lib \
  vendor/ORB_SLAM3/Thirdparty/g2o/lib \
  vendor/ORB_SLAM3/runtime-lib \
  install/orb_slam3_ros2/lib/orb_slam3_ros2 \
  install/orb_slam3_ros2/share/orb_slam3_ros2/config \
  install/orb_slam3_ros2/share/orb_slam3_ros2/launch \
  install/orb_slam3_ros2/share/ament_index/resource_index/packages

install -m 755 "$ART/lib/libORB_SLAM3.so" vendor/ORB_SLAM3/lib/libORB_SLAM3.so
install -m 755 "$ART/lib/libDBoW2.so" vendor/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
install -m 755 "$ART/lib/libg2o.so" vendor/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
install -m 755 "$ART/lib/libpango_core.so.0" vendor/ORB_SLAM3/runtime-lib/libpango_core.so.0
install -m 755 "$ART/lib/libpango_display.so.0" vendor/ORB_SLAM3/runtime-lib/libpango_display.so.0
install -m 755 "$ART/lib/libpango_opengl.so.0" vendor/ORB_SLAM3/runtime-lib/libpango_opengl.so.0
install -m 755 "$ART/lib/libpango_vars.so.0" vendor/ORB_SLAM3/runtime-lib/libpango_vars.so.0
install -m 755 "$ART/bin/orb_slam_node" install/orb_slam3_ros2/lib/orb_slam3_ros2/orb_slam_node
install -m 644 "$ART/config/ORBvoc.txt" vendor/ORBvoc.txt
install -m 644 src/orb_slam3_ros2/package.xml install/orb_slam3_ros2/share/orb_slam3_ros2/package.xml
cp -f src/orb_slam3_ros2/launch/*.py install/orb_slam3_ros2/share/orb_slam3_ros2/launch/
cp -f src/orb_slam3_ros2/config/* install/orb_slam3_ros2/share/orb_slam3_ros2/config/
touch install/orb_slam3_ros2/share/ament_index/resource_index/packages/orb_slam3_ros2
```

#### 3. Run the launch file
```bash
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/home/cesar/autonomy-stack/install/orb_slam3_ros2:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/home/cesar/autonomy-stack/vendor/ORB_SLAM3/lib:/home/cesar/autonomy-stack/vendor/ORB_SLAM3/Thirdparty/DBoW2/lib:/home/cesar/autonomy-stack/vendor/ORB_SLAM3/Thirdparty/g2o/lib:/home/cesar/autonomy-stack/vendor/ORB_SLAM3/runtime-lib:$LD_LIBRARY_PATH

ros2 launch orb_slam3_ros2 orb_slam_sim_launch.py
```

#### Notes
* The artifact bundle is built for Jetson-compatible Ubuntu 22.04 / ROS Humble ABI levels.
* If `ros2 launch` cannot find `orb_slam3_ros2`, re-run step 2 so the ament index file exists.
* If a library is missing at runtime, verify the files exist in `vendor/ORB_SLAM3/runtime-lib` and that `LD_LIBRARY_PATH` includes that directory.
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

## Test using bridge_stonefish
```sh
# TODO
```

## Build & Run proteus using bridge_hardware
```sh
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages (TODO: remember about detections stuff)
# VectorNav is vendored as a submodule under vendor/vectornav on the ros2 branch.
colcon build \
  --packages-select interfaces bringup mission_executor bridge_hardware vectornav vectornav_msgs \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Optional: disable VN-100 support at compile time
# (default is enabled)
colcon build \
  --packages-select bridge_hardware \
  --cmake-args -DBRIDGE_HARDWARE_ENABLE_VN100=OFF

# Optional: explicitly enable VN-100 support
colcon build \
  --packages-select bridge_hardware \
  --cmake-args -DBRIDGE_HARDWARE_ENABLE_VN100=ON

# When using hardware_proteus.launch.py, build bridge_hardware with VN-100 off
# because the IMU is started by the vectornav package launch file.

# Source the workspace
source install/setup.bash

# Run
ros2 launch bringup hardware_proteus.launch.py \
    mission_name:=prequalify \
    arduino_port:=/dev/ttyACM0 arduino_baud_rate:=115200

# VectorNav is started by bringup/hardware_proteus.launch.py and reads port/baud
# from vendor/vectornav/vectornav/config/vectornav.yaml.
```
