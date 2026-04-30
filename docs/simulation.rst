==========
Simulation
==========

We test the autonomy stack in simulation so that perception, planning, and
control can run together before moving to hardware.  The normal entry point is
the ``bringup`` Stonefish launch file, which starts the simulator, the mock
detection pipeline, joystick input, and, unless disabled, the mission executor.




Stonefish
=========

Stonefish is the simulator used by this repository.  Scenario files describe
the environment, robot, sensors, actuators, and objects, while ROS 2 launch
files connect the simulator to the rest of the autonomy stack.

Make sure Stonefish and the ROS 2 workspace have been built before launching a
simulation:

.. code-block:: sh

   source /opt/ros/jazzy/setup.bash

   colcon build \
     --packages-select interfaces bringup mission_executor bridge_stonefish Stonefish stonefish_ros2 detection_mocker joy sdl2_vendor

   source install/setup.bash




Launch File
============

Project launch file
-------------------

Use ``bringup/launch/stonefish.launch.py`` for normal mission simulations:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=prequalify \
     auv_name:=proteus \
     env_file_name:=proteus_env.scn \
     headless:=false

The launch file accepts these arguments:

``mission_name``
   Mission to run in ``mission_executor``.  Common values are
   ``prequalify`` and ``teleop``.

``auv_name``
   Robot configuration name.  The launch file has defaults for
   ``bluerov2``, ``proteus``, and ``hydrus``.

``env_file_name``
   Environment scenario file from
   ``src/bridge_stonefish/data/scenarios``.  When this is
   ``pool_env.scn``, the launch file automatically selects the robot-specific
   wrapper scenario: ``pool_env_bluerov2.scn``, ``pool_env_proteus.scn``, or
   ``pool_env_hydrus.scn``.

``auv_file_name``
   Optional robot scenario override.  Leave this empty for the default robot
   file selected from ``auv_name``.  Use it when testing a custom robot file,
   for example ``auv_file_name:=hydrus_auv_headless.scn``.

``headless``
   Set to ``true`` to run ``stonefish_simulator_nogpu`` without the graphical
   Stonefish window.  Set to ``false`` to run the graphical simulator.

``stonefish_only``
   Set to ``true`` when you only want the simulator, detection mocker, and
   joystick node.  This prevents ``mission_executor`` from starting, which is
   useful for actuator or topic-level tests.

The project launch file also passes fixed simulator settings to Stonefish:
``simulation_rate`` is ``300.0``.  In graphical mode the window is launched at
``1920x1080`` with ``rendering_quality`` set to ``high``.

Common configurations
---------------------

Proteus prequalification mission:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=prequalify \
     auv_name:=proteus \
     env_file_name:=proteus_env.scn \
     headless:=false

Hydrus prequalification mission:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=prequalify \
     auv_name:=hydrus \
     env_file_name:=hydrus_env.scn \
     headless:=false

BlueROV2 teleoperation in the pool environment:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=teleop \
     auv_name:=bluerov2 \
     env_file_name:=pool_env.scn \
     headless:=false

Run Stonefish without the mission executor for manual topic tests:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=teleop \
     auv_name:=bluerov2 \
     env_file_name:=pool_env.scn \
     headless:=false \
     stonefish_only:=true

Then publish commands from another sourced terminal.  For example, this sends
normalized thruster commands to the BlueROV2 bridge:

.. code-block:: sh

   source install/setup.bash

   ros2 topic pub -r 10 /bridge/thrusters std_msgs/msg/Float64MultiArray \
     "{data: [-0.6, -0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.0]}"

Direct Stonefish launch files
-----------------------------

For low-level scenario debugging, you can launch the Stonefish ROS 2 simulator
directly.  Use the graphical launch file when a GPU/OpenGL window is available:

.. code-block:: sh

   ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
     simulation_data:=$PWD/install/bridge_stonefish/share/bridge_stonefish/data \
     scenario_desc:=$PWD/install/bridge_stonefish/share/bridge_stonefish/data/scenarios/proteus_env.scn \
     simulation_rate:=300.0 \
     window_res_x:=1280 \
     window_res_y:=720 \
     rendering_quality:=high

Use the no-GPU launch file for headless runs:

.. code-block:: sh

   ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py \
     simulation_data:=$PWD/install/bridge_stonefish/share/bridge_stonefish/data \
     scenario_desc:=$PWD/install/bridge_stonefish/share/bridge_stonefish/data/scenarios/hydrus_env_headless.scn \
     simulation_rate:=300.0

The direct Stonefish launch files only start the simulator.  They do not start
``mission_executor``, ``detection_mocker``, or ``joy_node``.

Scenarios
=========

Scenario files live in ``src/bridge_stonefish/data/scenarios``.  The most
commonly used files are:

``proteus_env.scn``
   Complete Proteus environment.

``hydrus_env.scn``
   Complete Hydrus environment with graphical sensors.

``hydrus_env_headless.scn``
   Hydrus environment adjusted for headless/no-GPU execution.

``pool_env.scn``
   Generic pool selector used by the project launch file.  It resolves to a
   robot-specific pool wrapper based on ``auv_name``.

``pool_env_bluerov2.scn``, ``pool_env_proteus.scn``, ``pool_env_hydrus.scn``
   Pool environments with a specific robot included.

``bluerov2.scn``, ``proteus_auv.scn``, ``hydrus_auv.scn``
   Robot-only scenario fragments used as default AUV files.

To test a different configuration, change the launch parameters instead of
editing the launch file.  For example:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=teleop \
     auv_name:=hydrus \
     env_file_name:=pool_env.scn \
     auv_file_name:=hydrus_auv.scn \
     headless:=false

When adding a new robot or environment, place the new ``.scn`` file and its
meshes/textures under ``src/bridge_stonefish/data``.  If the robot should be a
standard option in the project launch file, add it to
``DEFAULT_AUV_FILE_BY_NAME`` and, for pool simulations, add the matching pool
wrapper to ``POOL_ENV_SCENARIO_BY_AUV`` in
``src/bringup/launch/stonefish.launch.py``.

Headless Runs
=============

Headless mode is useful for CI, smoke tests, and remote machines without a
display.  Prefer a headless scenario file when one exists:

.. code-block:: sh

   ros2 launch bringup stonefish.launch.py \
     mission_name:=prequalify \
     auv_name:=hydrus \
     env_file_name:=hydrus_env_headless.scn \
     auv_file_name:=hydrus_auv_headless.scn \
     headless:=true

Use ``stonefish_only:=true`` with ``headless:=true`` when you only need to
verify that the simulator starts and publishes ROS topics.
