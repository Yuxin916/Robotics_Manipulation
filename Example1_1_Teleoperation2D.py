# Imports
import numpy as np

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.geometry import MeshcatVisualizerCpp
from pydrake.manipulation.planner import (  
  DifferentialInverseKinematicsParameters, 
  DifferentialInverseKinematicsIntegrator )
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from manipulation import running_as_notebook
from manipulation.meshcat_cpp_utils import (
  StartMeshcat, MeshcatPoseSliders, WsgButton)

  # Start the visualizer.
meshcat = StartMeshcat(open_window=True)

def teleop_2d():
  builder = DiagramBuilder()

  station = builder.AddSystem(ManipulationStation())
  station.SetupPlanarIiwaStation()
  station.AddManipulandFromFile(
      "drake/examples/manipulation_station/models/"
      + "061_foam_brick.sdf",
      RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))
  # TODO(russt): Add planar joint to brick
  station.Finalize()

  visualizer = MeshcatVisualizerCpp.AddToBuilder(
      builder, station.GetOutputPort("query_object"), meshcat)
  meshcat.Set2dRenderMode()

  robot = station.get_controller_plant()
  params = DifferentialInverseKinematicsParameters(
      robot.num_positions(), robot.num_velocities())

  time_step = 0.005
  params.set_timestep(time_step)
  iiwa14_velocity_limits = np.array([1.4, 1.3, 2.3])
  params.set_joint_velocity_limits((-iiwa14_velocity_limits,
                                    iiwa14_velocity_limits))
  # These constants are in body frame.
  params.set_end_effector_velocity_gain([.1, 0, 0, 0, .1, .1])
  differential_ik = builder.AddSystem(
      DifferentialInverseKinematicsIntegrator(
          robot, robot.GetFrameByName("iiwa_link_7"), time_step, params))
  builder.Connect(differential_ik.get_output_port(),
                  station.GetInputPort("iiwa_position"))

  meshcat.DeleteAllButtonsAndSliders()
  teleop = builder.AddSystem(MeshcatPoseSliders(meshcat,
      min_range=MeshcatPoseSliders.MinRange(
          roll=0, x=-0.6, z=0.0),
      max_range=MeshcatPoseSliders.MaxRange(
          roll=2*np.pi, x=0.8, z=1.1),
      value=MeshcatPoseSliders.Value(pitch=0, yaw=0, y=0),
      visible=MeshcatPoseSliders.Visible(pitch=False, yaw=False, y=False) 
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

teleop_2d()