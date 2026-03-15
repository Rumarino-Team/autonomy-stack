
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

## Test mission_executor

### With GUI (local development)
```sh
# Navigate to the workspace
cd ~/ros2_ws/rumarino-ros2-jazzy

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build --packages-select interfaces bringup Stonefish stonefish_ros2 controller_stonefish  detection_mocker mission_executor

# Source the workspace
source install/setup.bash

# Run with GUI
ros2 launch bringup test_mission_executor.launch.py mission_name:=prequalify controller_name:=stonefish_hydrus env_file_name:=hydrus_env.scn
```

## Thruster Teleop

### Run simulator + teleop
```sh
# Terminal 1: launch simulator
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch controller_stonefish hydrussim.launch.py

# Terminal 2: run teleop controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run controller_stonefish thruster_teleop
```

### Keyboard controls
- `w/s`: increase/decrease x goal
- `r/f`: increase/decrease y goal
- `q/e`: increase/decrease z goal
- `a/d`: increase/decrease yaw goal
- `x`: hold current pose and reset PID history
- `z`: quit teleop

### PS5 controller support
`thruster_teleop` subscribes to `/joy` (`sensor_msgs/msg/Joy`).

```sh
# Terminal 3 (if using a controller)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run joy joy_node
```

Default mapping in teleop:
- Left stick Y: x goal
- Left stick X: y goal
- Right stick X: yaw goal
- L1 / R1: z up / z down
- Cross: hold current pose
- Circle: quit

### Use NVIDIA GPU for simulation
If you want to force NVIDIA offload for the simulator, run:

```sh
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only ros2 launch controller_stonefish hydrussim.launch.py
```


## Computer Vision

### Dependencies
  - [ZED-SDK 5.1](https://www.stereolabs.com/developers/release)
  - [Cuda 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive)

### Run ZED Custom Wrapper
```sh
colcon build --packages-select zed_msg zed_custom_wrapper && source ./install/setup.bash && ros2 launch zed_custom_wrapper zed_custom.launch.py onnx_model_path:=./src/zed_custom_wrapper/yolov8n.onnx
```
