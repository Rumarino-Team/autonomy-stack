========
Hardware
========

This page documents how to start the hardware-facing ROS nodes used by the
stack.  The main launch file is ``bringup/launch/hardware_proteus.launch.py``.
It can start the Proteus Arduino bridge, VectorNav IMU, USB camera, static
transforms, and ORB-SLAM3.

Before running any command, build and source the workspace:

.. code-block:: sh

   source /opt/ros/jazzy/setup.bash

   colcon build \
     --packages-select \
       interfaces bringup mission_executor bridge_hardware \
       vectornav vectornav_msgs orb_slam3_ros2 \
       zed_msgs zed_custom_wrapper \
     --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

   source install/setup.bash

Full Proteus Bringup
====================

Use this when running the hardware stack through the project launch file:

.. code-block:: sh

   ros2 launch bringup hardware_proteus.launch.py \
     mission_name:=prequalify \
     arduino_port:=/dev/ttyACM0 \
     arduino_baud_rate:=115200 \
     use_vectornav:=true \
     use_usb_cam:=false \
     use_orb_slam:=false

Important launch arguments:

``mission_name``
   Mission passed to ``mission_executor``.  Common values are ``prequalify``
   and ``teleop``.

``arduino_port``
   Serial device for the Proteus Arduino, for example ``/dev/ttyACM0``.

``arduino_baud_rate``
   Arduino serial baud rate.  The bridge defaults to ``115200``.

``use_vectornav``
   Starts ``vectornav.launch.py`` and the static transform from the vehicle
   base frame to the IMU frame.

``use_usb_cam``
   Starts ``usb_cam_node_exe`` and the static transform from the vehicle base
   frame to the camera frame.

``use_orb_slam``
   Starts ``orb_slam3_ros2`` with the USB camera image topic and VectorNav IMU
   topic.

``orb_use_viewer``
   Enables or disables the ORB-SLAM3 viewer.

``world_frame``, ``base_frame``, ``imu_frame``, ``camera_frame``
   Frame names used by the launch file and static transform publishers.

``imu_x``, ``imu_y``, ``imu_z``, ``imu_roll``, ``imu_pitch``, ``imu_yaw``
   Position and orientation of the IMU relative to ``base_frame``.

``camera_x`` / ``camera_y`` / ``camera_z`` and rotation arguments
   Position and orientation of the camera relative to ``base_frame``.

Proteus Arduino Bridge
======================

The Proteus hardware bridge is not a sensor, but it is the hardware actuator
interface that receives commands from the autonomy stack.

Launched by full bringup:

.. code-block:: sh

   ros2 launch bringup hardware_proteus.launch.py \
     mission_name:=teleop \
     arduino_port:=/dev/ttyACM0 \
     arduino_baud_rate:=115200

Run only the ROS node:

.. code-block:: sh

   ros2 run bridge_hardware bridge_proteus_node --ros-args \
     -p arduino_port:=/dev/ttyACM0 \
     -p arduino_baud_rate:=115200

Subscribed topics:

``/bridge/thrusters``
   ``std_msgs/msg/Float64MultiArray``.  Each value is sent to the Arduino as a
   thruster command.

Quick command test:

.. code-block:: sh

   ros2 topic pub -r 10 /bridge/thrusters std_msgs/msg/Float64MultiArray \
     "{data: [0.0, 0.0, 0.0, 0.0]}"

VectorNav IMU
=============

The VectorNav driver is included through ``vendor/vectornav``.  The default
configuration file is ``vendor/vectornav/vectornav/config/vectornav.yaml`` and
uses:

``port``
   ``/dev/ttyUSB0``

``baud``
   ``115200``

Launch with the full hardware stack:

.. code-block:: sh

   ros2 launch bringup hardware_proteus.launch.py \
     mission_name:=teleop \
     arduino_port:=/dev/ttyACM0 \
     arduino_baud_rate:=115200 \
     use_vectornav:=true

Launch only VectorNav:

.. code-block:: sh

   ros2 launch vectornav vectornav.launch.py

The VectorNav launch starts two nodes:

``vectornav``
   Reads the serial device and publishes raw VectorNav messages.

``vn_sensor_msgs``
   Converts raw VectorNav messages into standard ROS sensor messages.

Important topics:

``vectornav/raw/imu``
   ``vectornav_msgs/msg/ImuGroup`` raw IMU data.

``vectornav/imu``
   ``sensor_msgs/msg/Imu`` standard IMU output used by ORB-SLAM3.

``vectornav/gnss``
   ``sensor_msgs/msg/NavSatFix`` GNSS output when available.

``vectornav/magnetic``
   ``sensor_msgs/msg/MagneticField`` magnetometer output.

``vectornav/pressure``
   ``sensor_msgs/msg/FluidPressure`` pressure output.

Check the IMU:

.. code-block:: sh

   ros2 topic echo /vectornav/imu

USB Camera
==========

The hardware launch can start ``usb_cam`` for a regular V4L2 camera.

Launch with the full hardware stack:

.. code-block:: sh

   ros2 launch bringup hardware_proteus.launch.py \
     mission_name:=teleop \
     arduino_port:=/dev/ttyACM0 \
     arduino_baud_rate:=115200 \
     use_usb_cam:=true \
     camera_frame:=camera_link

Run only the camera node:

.. code-block:: sh

   ros2 run usb_cam usb_cam_node_exe --ros-args \
     -r __ns:=/usb_cam \
     -p video_device:=/dev/video0 \
     -p framerate:=30.0 \
     -p io_method:=mmap \
     -p frame_id:=camera_link \
     -p pixel_format:=mjpeg2rgb \
     -p image_width:=640 \
     -p image_height:=480 \
     -p camera_name:=proteus_usb_cam

Important topics:

``/usb_cam/image_raw``
   ``sensor_msgs/msg/Image`` image stream.

``/usb_cam/camera_info``
   ``sensor_msgs/msg/CameraInfo`` camera calibration metadata, when configured.

Check the camera:

.. code-block:: sh

   ros2 topic hz /usb_cam/image_raw
   ros2 topic echo /usb_cam/camera_info --once

USB Camera With ORB-SLAM3
=========================

Use this when the camera image and VectorNav IMU should feed ORB-SLAM3.

Launch through hardware bringup:

.. code-block:: sh

   ros2 launch bringup hardware_proteus.launch.py \
     mission_name:=teleop \
     arduino_port:=/dev/ttyACM0 \
     arduino_baud_rate:=115200 \
     use_vectornav:=true \
     use_usb_cam:=true \
     use_orb_slam:=true \
     orb_use_viewer:=false \
     camera_frame:=camera_link \
     imu_frame:=vectornav

Launch ORB-SLAM3 directly with hardware topics:

.. code-block:: sh

   ros2 launch orb_slam3_ros2 orb_slam_sim_launch.py \
     use_viewer:=false \
     use_imu:=true \
     use_depth:=false \
     image_topic:=/usb_cam/image_raw \
     imu_topic:=/vectornav/imu \
     settings_file:=$PWD/src/orb_slam3_ros2/config/webcamera.yaml \
     world_frame_id:=world \
     camera_frame_id:=camera_link \
     use_usb_cam:=true \
     use_vectornav:=true

ORB-SLAM3 node:

``orb_slam_node``
   Consumes image, optional depth, and optional IMU topics.  Publishes pose,
   odometry, path, and TF.

Important output topics:

``orb_slam3/camera_pose``
   ``geometry_msgs/msg/PoseStamped`` camera pose.

``orb_slam3/camera_odom``
   ``nav_msgs/msg/Odometry`` camera odometry.

``orb_slam3/camera_path``
   ``nav_msgs/msg/Path`` accumulated path.

ZED Camera
==========

The custom ZED wrapper starts the ZED SDK, optional custom object detection,
positional tracking, depth output, and a project map topic.

Launch the ZED wrapper:

.. code-block:: sh

   ros2 launch zed_custom_wrapper zed_custom.launch.py \
     onnx_model_path:=$PWD/src/zed_custom_wrapper/yolov8n.onnx

Launch from an SVO recording:

.. code-block:: sh

   ros2 launch zed_custom_wrapper zed_custom.launch.py \
     onnx_model_path:=$PWD/src/zed_custom_wrapper/yolov8n.onnx \
     svo_path:=/path/to/recording.svo

Run only the ZED ROS node:

.. code-block:: sh

   ros2 run zed_custom_wrapper zed_custom_node --ros-args \
     -p onnx_model_path:=$PWD/src/zed_custom_wrapper/yolov8n.onnx \
     -p svo_path:="" \
     -p resolution:=HD720 \
     -p fps:=30 \
     -p mapping_enabled:=true \
     -p matching_distance_threshold:=1.0

Important topics:

``zed/rgb/image_rect_color``
   ``sensor_msgs/msg/Image`` RGB stream.

``zed/depth/depth_registered``
   ``sensor_msgs/msg/Image`` registered depth stream.

``zed/objects``
   ``zed_msgs/msg/ObjectsStamped`` native ZED object detections.

``zed/odom``
   ``nav_msgs/msg/Odometry`` ZED positional tracking.

``zed/pose``
   ``geometry_msgs/msg/PoseStamped`` ZED pose.

``zed/map``
   ``interfaces/msg/Map`` object map generated from filtered detections.

``zed/map_markers``
   ``visualization_msgs/msg/MarkerArray`` tracked-object visualization.

Check ZED output:

.. code-block:: sh

   ros2 topic hz zed/rgb/image_rect_color
   ros2 topic echo zed/odom --once
   ros2 topic echo zed/map --once

Static Sensor Transforms
========================

The hardware launch publishes static transforms for the IMU and camera when
their corresponding sensors are enabled.  You can also run them manually.

Base to IMU:

.. code-block:: sh

   ros2 run tf2_ros static_transform_publisher \
     --frame-id base_link \
     --child-frame-id vectornav \
     --x 0.0 --y 0.0 --z 0.0 \
     --roll 0.0 --pitch 0.0 --yaw 0.0

Base to USB camera:

.. code-block:: sh

   ros2 run tf2_ros static_transform_publisher \
     --frame-id base_link \
     --child-frame-id camera_link \
     --x 0.0 --y 0.0 --z 0.0 \
     --roll 0.0 --pitch 0.0 --yaw 0.0

Preflight Checklist
===================

Before running a mission on hardware:

* Confirm the Arduino serial device path, usually ``/dev/ttyACM0``.
* Confirm the VectorNav serial device path in ``vectornav.yaml``, usually
  ``/dev/ttyUSB0``.
* Confirm the camera device path, usually ``/dev/video0``.
* Source ``install/setup.bash`` in every terminal.
* Check ``/bridge/thrusters`` with the vehicle safely disabled or out of water.
* Check sensor topics with ``ros2 topic list``, ``ros2 topic hz``, and
  ``ros2 topic echo`` before enabling autonomous control.
* Verify static transform frame names match the ORB-SLAM and sensor
  configuration files.
