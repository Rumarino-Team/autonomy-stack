# autonomy-stack

ROS 2 Humble simulation and control stack for Rumarino AUVs (Stonefish + `mission_executor`).

**Platform:** Ubuntu 22.04 (desktop and Jetson).

## Setup

```sh
git clone --recursive https://github.com/Rumarino-Team/autonomy-stack.git
cd autonomy-stack
./scripts/install_deps.sh
```

Optional flags: `--with-joy` (teleop), `--with-hardware` (Proteus USB cam / mock serial).

## Build

From the repo root:

```sh
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run simulation

Launch from the **repo root**. All missions, AUVs, and scenario options are defined in the launch file:

```sh
ros2 launch bringup stonefish.launch.py --show-args
```

Example:

```sh
ros2 launch bringup stonefish.launch.py mission_name:=prequalify auv_name:=hydrus headless:=false
```

Direct thruster test (no mission executor): add `stonefish_only:=true`, then publish to `/bridge/thrusters`.
