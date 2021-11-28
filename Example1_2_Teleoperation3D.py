def teleop_3d():
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())

    station.SetupClutterClearingStation()
    #ycb_objects = CreateClutterClearingYcbObjectList()
    #for model_file, X_WObject in ycb_objects:
    #    station.AddManipulandFromFile(model_file, X_WObject)
    station.AddManipulandFromFile(
        "drake/examples/manipulation_station/models/"
        + "061_foam_brick.sdf",
        RigidTransform(RotationMatrix.Identity(), [0, -0.6, 0.2]))
    station.Finalize()

    visualizer = MeshcatVisualizerCpp.AddToBuilder(
        builder, station.GetOutputPort("query_object"), meshcat)
    meshcat.ResetRenderMode()

    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(
        robot.num_positions(), robot.num_velocities())

    time_step = 0.005
    params.set_timestep(time_step)
    # True velocity limits for the IIWA14 (in rad, rounded down to the first
    # decimal)
    iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    params.set_joint_velocity_limits((-iiwa14_velocity_limits,
                                    iiwa14_velocity_limits))
    params.set_end_effector_velocity_gain([.1]*6)
    differential_ik = builder.AddSystem(DifferentialInverseKinematicsIntegrator(
            robot, robot.GetFrameByName("iiwa_link_7"), time_step, params))
    builder.Connect(differential_ik.get_output_port(),
                    station.GetInputPort("iiwa_position"))

    teleop = builder.AddSystem(MeshcatPoseSliders(meshcat,
        min_range = MeshcatPoseSliders.MinRange(
            roll=0, pitch=-0.5, yaw=-np.pi, x=-0.6, y=-0.8, z=0.0),
        max_range = MeshcatPoseSliders.MaxRange(
            roll=2*np.pi, pitch=np.pi, yaw=np.pi, x=0.8, y=0.3, z=1.1)
    ))
    builder.Connect(teleop.get_output_port(0), 
                    differential_ik.get_input_port())
    wsg_teleop = builder.AddSystem(WsgButton(meshcat))
    builder.Connect(wsg_teleop.get_output_port(0),
                    station.GetInputPort("wsg_position"))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()

    station_context = station.GetMyMutableContextFromRoot(context)

    q0 = station.GetOutputPort("iiwa_position_measured").Eval(
        station_context)
    differential_ik.get_mutable_parameters().set_nominal_joint_position(q0)
    diff_ik_context = differential_ik.GetMyMutableContextFromRoot(context)
    differential_ik.SetPositions(diff_ik_context, q0)
    teleop.SetPose(differential_ik.ForwardKinematics(diff_ik_context))

    if running_as_notebook:  # Then we're not just running as a test on CI.
        simulator.set_target_realtime_rate(1.0)

        meshcat.AddButton("Stop Simulation")
        while meshcat.GetButtonClicks("Stop Simulation") < 1:
            simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
        meshcat.DeleteButton("Stop Simulation")
    
    else:
        simulator.AdvanceTo(0.1)

teleop_3d()