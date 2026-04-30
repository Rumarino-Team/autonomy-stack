============
ROS Packages
============

The ROS packages in this repository are connected mostly through a small set of
topic interfaces.  The important idea is that perception publishes a map,
localization publishes odometry, ``mission_executor`` consumes both, and the
selected bridge sends the resulting thruster commands to either Stonefish or
real hardware.

Runtime Topic Graph
===================

.. mermaid::

   flowchart LR
     bringup["bringup<br/>launch files + config"]

     subgraph sim["Simulation path"]
       stonefish["stonefish_ros2<br/>Stonefish simulator"]
       bridge_assets["bridge_stonefish<br/>scenarios + assets"]
       mocker["detection_mocker<br/>scenario perception"]
     end

     subgraph hw["Hardware path"]
       proteus["bridge_hardware<br/>Proteus Arduino bridge"]
       vectornav["vectornav<br/>IMU driver"]
       usbcam["usb_cam<br/>camera driver"]
     end

     subgraph perception["Perception / localization"]
       zed["zed_custom_wrapper<br/>ZED camera + object map"]
       orb["orb_slam3_ros2<br/>visual / inertial odometry"]
     end

     interfaces["interfaces<br/>Map, MapObject, MovingState"]
     executor["mission_executor<br/>missions + PID + TAM"]
     joy["joy_node<br/>joystick"]
     arduino["Arduino / thrusters"]

     bringup --> stonefish
     bringup --> mocker
     bringup --> executor
     bringup --> proteus
     bringup --> vectornav
     bringup --> usbcam
     bringup --> orb

     bridge_assets --> stonefish
     stonefish -- "/bridge/odometry<br/>nav_msgs/Odometry" --> executor
     stonefish -- "/bridge/imu<br/>sensor_msgs/Imu" --> orb
     stonefish -- "camera topics<br/>sensor_msgs/Image" --> orb
     stonefish -- "scenario pose + objects" --> mocker
     mocker -- "/vision/map<br/>interfaces/Map" --> executor
     mocker -. "MarkerArray debug topics" .-> interfaces

     zed -- "zed/map<br/>interfaces/Map" --> interfaces
     zed -- "zed/odom<br/>nav_msgs/Odometry" --> interfaces
     zed -- "zed/rgb, zed/depth<br/>sensor_msgs/Image" --> orb
     zed -- "zed/objects<br/>zed_msgs/ObjectsStamped" --> interfaces
     orb -- "orb_slam3/camera_odom<br/>nav_msgs/Odometry" --> interfaces

     vectornav -- "/vectornav/imu<br/>sensor_msgs/Imu" --> orb
     usbcam -- "/usb_cam/image_raw<br/>sensor_msgs/Image" --> orb
     joy -- "/joy<br/>sensor_msgs/Joy" --> executor

     interfaces --> executor
     executor -- "/bridge/thrusters<br/>std_msgs/Float64MultiArray" --> stonefish
     executor -- "/bridge/thrusters<br/>std_msgs/Float64MultiArray" --> proteus
     proteus --> arduino

Custom Interfaces
=================

The ``interfaces`` package defines the project-specific messages that sit
between perception and mission planning.  Keep this package small and stable:
any change here affects the nodes that publish maps and the nodes that consume
maps.

``interfaces/msg/Map``
----------------------

.. code-block:: text

   vision_msgs/BoundingBox3D map_bounds
   MapObject[] objects

``Map`` is the world model passed to ``mission_executor``.  It contains:

``map_bounds``
   A 3D bounding box for the known operating area.  The executor uses this as
   context for navigation and collision checks.

``objects``
   The detected or known objects in the scene.  Each object is a
   ``MapObject``.

Current publishers:

``detection_mocker``
   Publishes ``/vision/map`` in simulation.  It builds this message by parsing
   Stonefish scenario objects and filtering them using vehicle odometry and
   detector parameters.

``bringup`` ``oneshot_map_node``
   Publishes one test ``/vision/map`` message for quick local testing.

``zed_custom_wrapper``
   Publishes ``zed/map`` from ZED object detections.  This is the real-camera
   equivalent of the simulated map, although the current ``mission_executor``
   subscribes to ``/vision/map`` by default.

Current subscribers:

``mission_executor``
   Subscribes to ``/vision/map``.  The selected mission reads object classes
   and bounding boxes, decides how to react, and updates the controller goal.

``interfaces/msg/MapObject``
----------------------------

.. code-block:: text

   int32 cls
   vision_msgs/BoundingBox3D bbox

``MapObject`` describes one semantic object in the map.

``cls``
   Integer class id.  In ``mission_executor`` these ids are interpreted as:
   ``0`` cube, ``1`` rectangle, ``2`` gate, ``3`` shark, ``4`` other, and
   ``5`` swordfish.

``bbox``
   Object pose and size as a ``vision_msgs/BoundingBox3D``.  The center pose is
   used as the target/reference point; the size is used for navigation and
   obstacle logic.

``interfaces/msg/MovingState``
------------------------------

.. code-block:: text

   bool depth
   bool rotation
   bool linear

``MovingState`` is a compact state flag message for motion status.  It is
defined in the interface package, but the current main runtime path does not
publish or subscribe to it yet.

Topic Contracts
===============

The table below shows the main topics, who publishes them, who subscribes to
them, and why they matter.

.. list-table::
   :header-rows: 1
   :widths: 24 24 24 28

   * - Topic
     - Message type
     - Publishers
     - Subscribers / use
   * - ``/vision/map``
     - ``interfaces/msg/Map``
     - ``detection_mocker``, ``oneshot_map_node``
     - ``mission_executor`` uses it for object-aware missions.
   * - ``zed/map``
     - ``interfaces/msg/Map``
     - ``zed_custom_wrapper``
     - Real-camera map output; remap to ``/vision/map`` when using it with
       ``mission_executor``.
   * - ``/bridge/odometry``
     - ``nav_msgs/msg/Odometry``
     - Stonefish scenario publishers
     - ``mission_executor`` uses it as the vehicle pose feedback.
   * - ``/bridge/thrusters``
     - ``std_msgs/msg/Float64MultiArray``
     - ``mission_executor`` or manual test publishers
     - Stonefish thruster subscribers and ``bridge_hardware`` consume it.
   * - ``/joy``
     - ``sensor_msgs/msg/Joy``
     - ``joy_node``
     - ``mission_executor`` consumes it in ``teleop`` mode.
   * - ``/bridge/imu``
     - ``sensor_msgs/msg/Imu``
     - Stonefish scenario publishers
     - ORB-SLAM or visualization tools can consume it.
   * - ``/vectornav/imu``
     - ``sensor_msgs/msg/Imu``
     - ``vectornav``
     - Hardware ORB-SLAM launch consumes it when IMU mode is enabled.
   * - ``/usb_cam/image_raw``
     - ``sensor_msgs/msg/Image``
     - ``usb_cam``
     - ``orb_slam3_ros2`` consumes it in hardware camera mode.
   * - ``/hydrus/rgb_camera/image_raw``
     - ``sensor_msgs/msg/Image``
     - Stonefish Hydrus scenario
     - ``orb_slam3_ros2`` default simulation image input.
   * - ``/hydrus/depth_camera/image_raw``
     - ``sensor_msgs/msg/Image``
     - Stonefish Hydrus scenario
     - ``orb_slam3_ros2`` default simulation depth input.
   * - ``orb_slam3/camera_pose``
     - ``geometry_msgs/msg/PoseStamped``
     - ``orb_slam3_ros2``
     - Debug/localization output for consumers that need camera pose.
   * - ``orb_slam3/camera_odom``
     - ``nav_msgs/msg/Odometry``
     - ``orb_slam3_ros2``
     - Visual odometry output; can be remapped into the control path.
   * - ``orb_slam3/camera_path``
     - ``nav_msgs/msg/Path``
     - ``orb_slam3_ros2``
     - RViz/path visualization.
   * - ``zed/rgb/image_rect_color``
     - ``sensor_msgs/msg/Image``
     - ``zed_custom_wrapper``
     - Camera stream for visualization or visual odometry.
   * - ``zed/depth/depth_registered``
     - ``sensor_msgs/msg/Image``
     - ``zed_custom_wrapper``
     - Depth stream for RGB-D perception/localization.
   * - ``zed/objects``
     - ``zed_msgs/msg/ObjectsStamped``
     - ``zed_custom_wrapper``
     - Native ZED detections for debugging or ZED-specific consumers.
   * - ``zed/odom``
     - ``nav_msgs/msg/Odometry``
     - ``zed_custom_wrapper``
     - ZED positional tracking output.
   * - ``zed/pose``
     - ``geometry_msgs/msg/PoseStamped``
     - ``zed_custom_wrapper``
     - ZED pose output.
   * - ``/torpedo/push``
     - ``std_msgs/msg/Float64``
     - ``bridge_stonefish launch_torpedo``
     - Stonefish torpedo object receives push force.

Simulation Interface Flow
=========================

.. mermaid::

   sequenceDiagram
     participant S as stonefish_ros2
     participant D as detection_mocker
     participant I as interfaces/Map
     participant M as mission_executor
     participant B as Stonefish thrusters

     S->>M: /bridge/odometry (nav_msgs/Odometry)
     S->>D: odometry topic configured by bringup
     D->>I: build Map from scenario + robot pose
     I->>M: /vision/map (interfaces/Map)
     M->>M: mission selects goal and PID computes thrusters
     M->>B: /bridge/thrusters (Float64MultiArray)

In simulation, the map normally comes from ``detection_mocker``.  The robot
state comes from the Stonefish scenario publishers.  ``mission_executor`` joins
those two streams and publishes thruster commands back into Stonefish.

Hardware Interface Flow
=======================

.. mermaid::

   sequenceDiagram
     participant C as camera/vision stack
     participant V as vectornav / odometry source
     participant I as interfaces/Map
     participant M as mission_executor
     participant H as bridge_hardware
     participant A as Arduino

     C->>I: /vision/map or remapped zed/map (interfaces/Map)
     V->>M: /bridge/odometry or remapped odometry
     I->>M: object map
     M->>H: /bridge/thrusters (Float64MultiArray)
     H->>A: serial thruster commands

On hardware, the control contract stays the same: ``mission_executor`` still
expects a map-like perception topic and odometry feedback, and it still outputs
``/bridge/thrusters``.  The difference is that ``bridge_hardware`` converts
those commands into serial messages for the Arduino instead of Stonefish
consuming them directly.

Package Responsibilities
========================

``bringup``
-----------

Owns launch files and shared configuration.

``stonefish.launch.py``
   Starts the simulator, ``detection_mocker``, ``joy_node``, and usually
   ``mission_executor``.

``hardware_proteus.launch.py``
   Starts ``mission_executor``, ``bridge_hardware``, and optional hardware
   sensors such as VectorNav, USB camera, and ORB-SLAM.

``config/mission_executor.toml``
   Holds PID gains and the thruster allocation matrix used by
   ``mission_executor``.

``mission_executor``
--------------------

Consumes ``/vision/map``, ``/bridge/odometry``, and optionally ``/joy``.  It
runs the selected mission, computes a goal, applies PID control and the
thruster allocation matrix, then publishes ``/bridge/thrusters``.

``bridge_stonefish``
--------------------

Packages Stonefish scenario files, meshes, textures, and helper commands.  The
scenario files declare Stonefish ROS publishers/subscribers such as
``/bridge/odometry``, ``/bridge/imu``, camera topics, and
``/bridge/thrusters``.

``detection_mocker``
--------------------

Turns Stonefish scenario objects into ``interfaces/msg/Map`` output.  It is the
simulation stand-in for real object detection.

``bridge_hardware``
-------------------

Subscribes to ``/bridge/thrusters`` and forwards each normalized thruster value
to the Proteus Arduino over serial.

``orb_slam3_ros2``
------------------

Consumes image, depth, and/or IMU topics.  Publishes camera pose, odometry, and
path topics for visual/inertial localization.

``zed_custom_wrapper``
----------------------

Publishes ZED RGB/depth images, ZED native detections, ZED pose/odometry, and
a project ``interfaces/msg/Map`` topic named ``zed/map``.

Vendored Packages
=================

``stonefish_ros2``
   ROS 2 integration for Stonefish, including graphical and headless simulator
   launch files plus Stonefish-specific messages/services.

``vectornav`` and ``vectornav_msgs``
   Hardware IMU/GNSS driver and message definitions.

``zed_msgs``
   Native ZED message and service definitions used by ``zed_custom_wrapper``.

``stonefish_bluerov2``
   External Stonefish examples and assets used as references for additional
   robot and environment scenarios.

