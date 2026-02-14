# Detection Mocker

ROS2 package for mocking object detection from Stonefish simulation scene files.

## Nodes

### 1. detection_mocker

Main node that publishes detected objects based on camera frustum culling.

**Subscribed Topics:**
- `odometry_topic` (nav_msgs/Odometry): Robot odometry for frustum culling

**Published Topics:**
- `map_output_topic` (interfaces/Map): Detected objects within camera view

**Parameters:**
- `scn_file_path` (string): Path to environment .scn file
- `robot_scn_file_path` (string): Path to robot .scn file (for camera specs)
- `odometry_topic` (string, default: "/hydrus/odometry"): Odometry topic
- `map_output_topic` (string, default: "/map"): Output map topic
- `publish_rate_hz` (double, default: 10.0): Publishing rate
- `min_detection_distance` (double, default: 0.1): Minimum detection range (m)
- `max_detection_distance` (double, default: 50.0): Maximum detection range (m)
- `publish_all_objects` (bool, default: false): Bypass frustum culling for debugging

**Launch:**
```bash
ros2 launch detection_mocker detection_mocker.launch.py
```

## Object Classification

Objects are classified by name matching:
- `cls=0`: Buoy (contains "buoy")
- `cls=1`: Pipe (contains "pipe")
- `cls=2`: Gate (contains "gate")
- `cls=3`: Unknown (everything else)

## Technical Details

- Uses TinyXML2 system library for parsing Stonefish .scn files
- Implements camera frustum culling with configurable FOV
- Supports BOX, CYLINDER, MODEL, and PLANE object types
- Transient local QoS for late-joining subscribers
- Automatic camera configuration from robot .scn file

## Fix Notes

The segfault issue was resolved by switching from bundled tinyxml2.cpp to the system TinyXML2 library. The bundled version had a conflict with rclcpp during static initialization.
