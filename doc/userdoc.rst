:github_url: https://github.com/RKinDLab/Reach_alpha_Blue_heavy_Sim/blob/master/doc/userdoc.rst

.. _ros2_control_RA5BHS_userdoc:

************************************************
Reach Alpha 5 with multiple interfaces
************************************************

For *reach alpha 5*, the hardware interface plugin is implemented having multiple interfaces.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints can be exchanged at once or independently.

## Dependencies

.. code-block:: shell
cd ~/your_workspace/src
git clone https://github.com/edxmorgan/tf2_broadcaster


Tutorial steps
--------------------------

1. To check that *reach alpha 5* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 view_robot.launch.py

   .. note::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *reach alpha 5* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_blue_reach_5 robot_system_multi_interface.launch.py use_mock_hardware:=true

   Useful launch-file options:

   ``robot_controller:=forward_position_controller``
    starts demo and spawns position controller. Robot can be then controlled using ``forward_position_controller`` as described below.

   ``robot_controller:=forward_current_controller``
    starts demo and spawns current controller. Robot can be then controlled using ``forward_current_controller`` as described below.

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purposes and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *reach alpha 5*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   .. code-block:: shell

    command interfaces
         alpha_axis_a/current [available] [unclaimed]
         alpha_axis_a/effort [available] [claimed]
         alpha_axis_a/position [available] [unclaimed]
         alpha_axis_a/velocity [available] [unclaimed]
         alpha_axis_b/current [available] [unclaimed]
         alpha_axis_b/effort [available] [claimed]
         alpha_axis_b/position [available] [unclaimed]
         alpha_axis_b/velocity [available] [unclaimed]
         alpha_axis_c/current [available] [unclaimed]
         alpha_axis_c/effort [available] [claimed]
         alpha_axis_c/position [available] [unclaimed]
         alpha_axis_c/velocity [available] [unclaimed]
         alpha_axis_d/current [available] [unclaimed]
         alpha_axis_d/effort [available] [claimed]
         alpha_axis_d/position [available] [unclaimed]
         alpha_axis_d/velocity [available] [unclaimed]
         alpha_axis_e/current [available] [unclaimed]
         alpha_axis_e/effort [available] [claimed]
         alpha_axis_e/position [available] [unclaimed]
         alpha_axis_e/velocity [available] [unclaimed]
         alphathruster1_joint/current [available] [unclaimed]
         alphathruster1_joint/effort [available] [claimed]
         alphathruster1_joint/velocity [available] [unclaimed]
         alphathruster2_joint/current [available] [unclaimed]
         alphathruster2_joint/effort [available] [claimed]
         alphathruster2_joint/velocity [available] [unclaimed]
         alphathruster3_joint/current [available] [unclaimed]
         alphathruster3_joint/effort [available] [claimed]
         alphathruster3_joint/velocity [available] [unclaimed]
         alphathruster4_joint/current [available] [unclaimed]
         alphathruster4_joint/effort [available] [claimed]
         alphathruster4_joint/velocity [available] [unclaimed]
         alphathruster5_joint/current [available] [unclaimed]
         alphathruster5_joint/effort [available] [claimed]
         alphathruster5_joint/velocity [available] [unclaimed]
         alphathruster6_joint/current [available] [unclaimed]
         alphathruster6_joint/effort [available] [claimed]
         alphathruster6_joint/velocity [available] [unclaimed]
         alphathruster7_joint/current [available] [unclaimed]
         alphathruster7_joint/effort [available] [claimed]
         alphathruster7_joint/velocity [available] [unclaimed]
         alphathruster8_joint/current [available] [unclaimed]
         alphathruster8_joint/effort [available] [claimed]
         alphathruster8_joint/velocity [available] [unclaimed]

    state interfaces
         alpha_axis_a/acceleration
         alpha_axis_a/current
         alpha_axis_a/effort
         alpha_axis_a/estimated_acceleration
         alpha_axis_a/filtered_position
         alpha_axis_a/filtered_velocity
         alpha_axis_a/position
         alpha_axis_a/stateId
         alpha_axis_a/velocity
         alpha_axis_b/acceleration
         alpha_axis_b/current
         alpha_axis_b/effort
         alpha_axis_b/estimated_acceleration
         alpha_axis_b/filtered_position
         alpha_axis_b/filtered_velocity
         alpha_axis_b/position
         alpha_axis_b/stateId
         alpha_axis_b/velocity
         alpha_axis_c/acceleration
         alpha_axis_c/current
         alpha_axis_c/effort
         alpha_axis_c/estimated_acceleration
         alpha_axis_c/filtered_position
         alpha_axis_c/filtered_velocity
         alpha_axis_c/position
         alpha_axis_c/stateId
         alpha_axis_c/velocity
         alpha_axis_d/acceleration
         alpha_axis_d/current
         alpha_axis_d/effort
         alpha_axis_d/estimated_acceleration
         alpha_axis_d/filtered_position
         alpha_axis_d/filtered_velocity
         alpha_axis_d/position
         alpha_axis_d/stateId
         alpha_axis_d/velocity
         alpha_axis_e/acceleration
         alpha_axis_e/current
         alpha_axis_e/effort
         alpha_axis_e/estimated_acceleration
         alpha_axis_e/filtered_position
         alpha_axis_e/filtered_velocity
         alpha_axis_e/position
         alpha_axis_e/stateId
         alpha_axis_e/velocity
         alphaimu_sensor/orientation.w
         alphaimu_sensor/orientation.x
         alphaimu_sensor/orientation.y
         alphaimu_sensor/orientation.z
         alphaimu_sensor/position.x
         alphaimu_sensor/position.y
         alphaimu_sensor/position.z
         alphaimu_sensor/velocity.p
         alphaimu_sensor/velocity.q
         alphaimu_sensor/velocity.r
         alphaimu_sensor/velocity.u
         alphaimu_sensor/velocity.v
         alphaimu_sensor/velocity.w
         alphathruster1_joint/acceleration
         alphathruster1_joint/current
         alphathruster1_joint/effort
         alphathruster1_joint/position
         alphathruster1_joint/velocity
         alphathruster2_joint/acceleration
         alphathruster2_joint/current
         alphathruster2_joint/effort
         alphathruster2_joint/position
         alphathruster2_joint/velocity
         alphathruster3_joint/acceleration
         alphathruster3_joint/current
         alphathruster3_joint/effort
         alphathruster3_joint/position
         alphathruster3_joint/velocity
         alphathruster4_joint/acceleration
         alphathruster4_joint/current
         alphathruster4_joint/effort
         alphathruster4_joint/position
         alphathruster4_joint/velocity
         alphathruster5_joint/acceleration
         alphathruster5_joint/current
         alphathruster5_joint/effort
         alphathruster5_joint/position
         alphathruster5_joint/velocity
         alphathruster6_joint/acceleration
         alphathruster6_joint/current
         alphathruster6_joint/effort
         alphathruster6_joint/position
         alphathruster6_joint/velocity
         alphathruster7_joint/acceleration
         alphathruster7_joint/current
         alphathruster7_joint/effort
         alphathruster7_joint/position
         alphathruster7_joint/velocity
         alphathruster8_joint/acceleration
         alphathruster8_joint/current
         alphathruster8_joint/effort
         alphathruster8_joint/position
         alphathruster8_joint/velocity

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

4. Check which controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   gives

   .. code-block:: shell

      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
      tf2_broadcaster     [tf2_broadcaster/Tf2Broadcaster] active    
      forward_effort_controller[forward_command_controller/ForwardCommandController] active

   Check how this output changes if you use the different launch file arguments described above.

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ROS 2 CLI interface.

      * when using ``forward_effort_controller`` controller

        .. code-block:: shell

         ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

      * when using ``forward_position_controller`` controller

        .. code-block:: shell

         ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, 2.4, 3.0, 0.5, 2.1,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

      * when using ``forward_velocity_controller`` controller (default)

        .. code-block:: shell

         ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.1, 0.2, 0.1 ,0.5, 0.1,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once