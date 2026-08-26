#!/usr/bin/env bash
# Install system dependencies for autonomy-stack (Ubuntu 22.04 + ROS 2 Humble).
set -euo pipefail

ROS_DISTRO=humble
WITH_JOY=false
WITH_HARDWARE=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Install build tools, ROS 2 Humble, simulation libraries, and Rust.

Options:
  --with-joy        Also install ros-${ROS_DISTRO}-joy and xterm (teleop)
  --with-hardware   Also install ros-${ROS_DISTRO}-usb-cam and socat (Proteus hardware)
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-joy) WITH_JOY=true ;;
        --with-hardware) WITH_HARDWARE=true ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
    shift
done

if ! command -v apt-get >/dev/null 2>&1; then
    echo "This script supports Ubuntu (apt) only." >&2
    exit 1
fi

ros_base_pkg="ros-${ROS_DISTRO}-ros-base"
if ! dpkg -s "$ros_base_pkg" >/dev/null 2>&1; then
    echo "Setting up ROS 2 ${ROS_DISTRO} apt repository..."
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
else
    echo "ROS 2 ${ROS_DISTRO} already installed; skipping repository setup."
fi

echo "Installing build tools, ROS 2, and simulation libraries..."
sudo apt update
sudo apt install -y \
    python3 python3-pip build-essential cmake pkg-config curl \
    libclang-dev llvm-dev clang \
    python3-colcon-common-extensions \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-vision-msgs" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-visualization-msgs" \
    libfreetype6-dev \
    libglm-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libgl1-mesa-dev \
    libsdl2-dev

if [[ "$WITH_JOY" == true ]]; then
    sudo apt install -y "ros-${ROS_DISTRO}-joy" xterm
fi

if [[ "$WITH_HARDWARE" == true ]]; then
    sudo apt install -y "ros-${ROS_DISTRO}-usb-cam" socat
fi

if ! command -v rustc >/dev/null 2>&1; then
    echo "Installing Rust (required for mission_executor)..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    # shellcheck disable=SC1091
    source "$HOME/.cargo/env"
else
    echo "Rust already installed; skipping rustup."
fi

echo "Done. Source ROS and build from the repo root:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "  source install/setup.bash"
echo "  ros2 launch bringup stonefish.launch.py --show-args"
