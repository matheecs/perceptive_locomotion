from distutils.command.build import build
import time
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    MeshcatVisualizer,
    RigidTransform,
    CoulombFriction,
    ConnectContactResultsToDrakeVisualizer,
    Simulator,
    PidController,
    HalfSpace,
    MeshcatContactVisualizer,
)
from pydrake.systems.primitives import LogVectorOutput


from matplotlib import pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz


def set_home(plant, context):
    hip_x = 0.0
    hip_y = 0.4
    knee = -0.8
    plant.GetJointByName("fr.hx").set_angle(context, hip_x)
    plant.GetJointByName("fr.hy").set_angle(context, hip_y)
    plant.GetJointByName("fr.kn").set_angle(context, knee)
    plant.GetJointByName("fl.hx").set_angle(context, hip_x)
    plant.GetJointByName("fl.hy").set_angle(context, hip_y)
    plant.GetJointByName("fl.kn").set_angle(context, knee)
    plant.GetJointByName("hl.hx").set_angle(context, hip_x)
    plant.GetJointByName("hl.hy").set_angle(context, hip_y)
    plant.GetJointByName("hl.kn").set_angle(context, knee)
    plant.GetJointByName("hr.hx").set_angle(context, hip_x)
    plant.GetJointByName("hr.hy").set_angle(context, hip_y)
    plant.GetJointByName("hr.kn").set_angle(context, knee)
    plant.SetFreeBodyPose(
        context, plant.GetBodyByName("base"), RigidTransform([0, 0, 1.9])
    )


builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-4)
parser = Parser(plant)
parser.AddModelFromFile("spot.urdf")
plant.RegisterVisualGeometry(
    plant.world_body(),
    RigidTransform.Identity(),
    HalfSpace(),
    "GroundVisualGeometry",
    [0.5, 0.5, 0.5, 1.0],
)
plant.set_penetration_allowance(0.001)
plant.RegisterCollisionGeometry(
    plant.world_body(),
    RigidTransform.Identity(),
    HalfSpace(),
    "GroundCollisionGeometry",
    CoulombFriction(1.0, 1.0),
)
plant.Finalize()

# Add a PD Controller
# kp = 200.0 * np.ones(12)
# ki = 40.0 * np.ones(12)
# kd = 50.0 * np.ones(12)

kp = 0 * np.ones(12)
ki = 0 * np.ones(12)
kd = 0 * np.ones(12)
# kd[-4:] = 0.16  # use lower gain for the knee joints
# Select the joint states (and ignore the floating-base states)
S = np.zeros((24, 37))
S[:12, 7:19] = np.eye(12)
S[12:, 25:] = np.eye(12)
pid_controller = builder.AddSystem(
    PidController(
        kp=kp,
        ki=ki,
        kd=kd,
        state_projection=S,
        output_projection=plant.MakeActuationMatrix()[6:, :].T,
    )
)

builder.Connect(
    plant.get_state_output_port(), pid_controller.get_input_port_estimated_state()
)
builder.Connect(pid_controller.get_output_port(), plant.get_actuation_input_port())

meshcat_vis = builder.AddSystem(
    MeshcatVisualizer(scene_graph, zmq_url="new", open_browser=True)
)
builder.Connect(
    scene_graph.get_query_output_port(), meshcat_vis.get_geometry_query_input_port()
)

contact_viz = builder.AddSystem(
    MeshcatContactVisualizer(
        meshcat_viz=meshcat_vis,
        force_threshold=0,
        contact_force_scale=1,
        plant=plant,
        contact_force_radius=0.005,
    )
)
contact_input_port = contact_viz.GetInputPort("contact_results")
builder.Connect(plant.GetOutputPort("contact_results"), contact_input_port)


diagram = builder.Build()


#
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyContextFromRoot(context)
set_home(plant, plant_context)
x0 = S @ plant.get_state_output_port().Eval(plant_context)
pid_controller.get_input_port_desired_state().FixValue(
    pid_controller.GetMyContextFromRoot(context), x0
)

# print(context)

meshcat_vis.reset_recording()
meshcat_vis.start_recording()

simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(2)

meshcat_vis.stop_recording()
meshcat_vis.publish_recording()
meshcat_vis.vis.render_static()
