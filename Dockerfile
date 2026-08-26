# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    build-essential \
    curl \
    cmake \
    git \
    pkg-config \
    libclang-dev \
    llvm-dev \
    clang \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    libfreetype6-dev \
    libsdl2-dev \
    libglm-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
ENV CARGO_TARGET_DIR=/ros2_ws/target

WORKDIR /ros2_ws

# Stonefish + package manifests (layer cache)
COPY vendor/stonefish ./vendor/stonefish
COPY src/interfaces/package.xml ./src/interfaces/package.xml
COPY src/bringup/package.xml ./src/bringup/package.xml
COPY src/bridge_stonefish/package.xml ./src/bridge_stonefish/package.xml
COPY src/mission_executor/package.xml ./src/mission_executor/package.xml
COPY src/detection_mocker/package.xml ./src/detection_mocker/package.xml
COPY vendor/stonefish_ros2/package.xml ./src/stonefish_ros2/package.xml

RUN rosdep init || true && rosdep update

RUN bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src vendor/stonefish --ignore-src -r -y || true"

COPY src/interfaces ./src/interfaces
RUN bash -lc "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select Stonefish interfaces \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY Cargo.toml Cargo.lock ./
COPY src/mission_executor/Cargo.toml ./src/mission_executor/Cargo.toml
COPY src/mission_executor/CMakeLists.txt ./src/mission_executor/CMakeLists.txt
RUN mkdir -p src/mission_executor/src && printf "fn main() {}" > src/mission_executor/src/main.rs

RUN bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && \
    colcon build --packages-select mission_executor \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY src/bringup ./src/bringup
COPY src/bridge_stonefish ./src/bridge_stonefish
COPY src/detection_mocker ./src/detection_mocker
COPY vendor/stonefish_ros2 ./src/stonefish_ros2

RUN bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && \
    colcon build --packages-select stonefish_ros2 bridge_stonefish bringup detection_mocker \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY src/mission_executor/src ./src/mission_executor/src
RUN ln -sf /ros2_ws/src/mission_executor/target /ros2_ws/target && \
    bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && \
    colcon build --packages-select mission_executor \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

ENV ROS_DOMAIN_ID=0
