:github_url: https://github.com/RKinDLab/Reach_alpha_Blue_heavy_Sim/blob/master/doc/userdoc.rst

.. _ros2_control_RA5BHS_userdoc:

************************************************
Reach Alpha Blue heavy Simulator
************************************************

For *Reach Alpha Blue heavy Simulator*, the hardware interface plugin is implemented having multiple interfaces.


Tutorial steps
--------------------------

1. To check that *reach alpha blue heavy sim* descriptions are working properly use following launch commands

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

   ``robot_controller:=forward_effort_controller``
    starts demo and spawns effort controller. Robot can be then controlled using ``forward_effort_controller`` as described below.

   ``robot_controller:=forward_current_controller``
    starts demo and spawns current controller. Robot can be then controlled using ``forward_current_controller`` as described below.

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   .. code-block:: shell

      command interfaces
         alpha_axis_a/current [available] [claimed]
         alpha_axis_a/effort [available] [unclaimed]
         alpha_axis_a/position [available] [unclaimed]
         alpha_axis_a/velocity [available] [unclaimed]
         alpha_axis_b/current [available] [claimed]
         alpha_axis_b/effort [available] [unclaimed]
         alpha_axis_b/position [available] [unclaimed]
         alpha_axis_b/velocity [available] [unclaimed]
         alpha_axis_c/current [available] [claimed]
         alpha_axis_c/effort [available] [unclaimed]
         alpha_axis_c/position [available] [unclaimed]
         alpha_axis_c/velocity [available] [unclaimed]
         alpha_axis_d/current [available] [claimed]
         alpha_axis_d/effort [available] [unclaimed]
         alpha_axis_d/position [available] [unclaimed]
         alpha_axis_d/velocity [available] [unclaimed]
         alpha_axis_e/current [available] [claimed]
         alpha_axis_e/effort [available] [unclaimed]
         alpha_axis_e/position [available] [unclaimed]
         alpha_axis_e/velocity [available] [unclaimed]
         alphathruster1_joint/current [available] [claimed]
         alphathruster1_joint/effort [available] [unclaimed]
         alphathruster2_joint/current [available] [claimed]
         alphathruster2_joint/effort [available] [unclaimed]
         alphathruster3_joint/current [available] [claimed]
         alphathruster3_joint/effort [available] [unclaimed]
         alphathruster4_joint/current [available] [claimed]
         alphathruster4_joint/effort [available] [unclaimed]
         alphathruster5_joint/current [available] [claimed]
         alphathruster5_joint/effort [available] [unclaimed]
         alphathruster6_joint/current [available] [claimed]
         alphathruster6_joint/effort [available] [unclaimed]
         alphathruster7_joint/current [available] [claimed]
         alphathruster7_joint/effort [available] [unclaimed]
         alphathruster8_joint/current [available] [claimed]
         alphathruster8_joint/effort [available] [unclaimed]

      state interfaces
         alpha_axis_a/acceleration
         alpha_axis_a/current
         alpha_axis_a/effort
         alpha_axis_a/estimated_acceleration
         alpha_axis_a/estimated_effort
         alpha_axis_a/estimated_inertia_zz
         alpha_axis_a/filtered_position
         alpha_axis_a/filtered_velocity
         alpha_axis_a/position
         alpha_axis_a/stateId
         alpha_axis_a/velocity
         alpha_axis_b/acceleration
         alpha_axis_b/current
         alpha_axis_b/effort
         alpha_axis_b/estimated_acceleration
         alpha_axis_b/estimated_effort
         alpha_axis_b/estimated_inertia_zz
         alpha_axis_b/filtered_position
         alpha_axis_b/filtered_velocity
         alpha_axis_b/position
         alpha_axis_b/stateId
         alpha_axis_b/velocity
         alpha_axis_c/acceleration
         alpha_axis_c/current
         alpha_axis_c/effort
         alpha_axis_c/estimated_acceleration
         alpha_axis_c/estimated_effort
         alpha_axis_c/estimated_inertia_zz
         alpha_axis_c/filtered_position
         alpha_axis_c/filtered_velocity
         alpha_axis_c/position
         alpha_axis_c/stateId
         alpha_axis_c/velocity
         alpha_axis_d/acceleration
         alpha_axis_d/current
         alpha_axis_d/effort
         alpha_axis_d/estimated_acceleration
         alpha_axis_d/estimated_effort
         alpha_axis_d/estimated_inertia_zz
         alpha_axis_d/filtered_position
         alpha_axis_d/filtered_velocity
         alpha_axis_d/position
         alpha_axis_d/stateId
         alpha_axis_d/velocity
         alpha_axis_e/acceleration
         alpha_axis_e/current
         alpha_axis_e/effort
         alpha_axis_e/estimated_acceleration
         alpha_axis_e/estimated_effort
         alpha_axis_e/estimated_inertia_zz
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


   Marker ``[claimed]`` by command interfaces means that a controller has access to command *system*.

4. Check which controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   gives

   .. code-block:: shell

      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
      forward_current_controller[forward_command_controller/ForwardCommandController] active

   Check how this output changes if you use the different launch file arguments described above.

5. If you get output from above you can send commands to *Forward Current Controller*, either:

   #. Manually using ROS 2 CLI interface.

      * when using ``forward_current_controller`` controller

        .. code-block:: shell

         ros2 topic pub /forward_current_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once

      * when using ``forward_effort_controller`` controller

        .. code-block:: shell

         ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0]}" --once
   
   .. note::
      The initial five floating-point values are assigned sequentially to the manipulator, starting from the base at index[0] to the end-effector 
      at index[4]. The subsequent eight floating-point values are designated for the vehicle's thrusters.
