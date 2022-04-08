from distutils.command.build import build
import time
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    MeshcatVisualizer,
    RigidTransform,
    SpatialVelocity,
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
from BalanceController import BalanceController


def set_home(plant, context):
    hip_x = 0.0
    hip_y = 0.4
    knee = -0.8
    plant.GetJointByName("fl.hx").set_angle(context, hip_x)
    plant.GetJointByName("fl.hy").set_angle(context, hip_y)
    plant.GetJointByName("fl.kn").set_angle(context, knee)
    plant.GetJointByName("fr.hx").set_angle(context, hip_x)
    plant.GetJointByName("fr.hy").set_angle(context, hip_y)
    plant.GetJointByName("fr.kn").set_angle(context, knee)
    plant.GetJointByName("hl.hx").set_angle(context, hip_x)
    plant.GetJointByName("hl.hy").set_angle(context, hip_y)
    plant.GetJointByName("hl.kn").set_angle(context, knee)
    plant.GetJointByName("hr.hx").set_angle(context, hip_x)
    plant.GetJointByName("hr.hy").set_angle(context, hip_y)
    plant.GetJointByName("hr.kn").set_angle(context, knee)
    plant.SetFreeBodyPose(
        context, plant.GetBodyByName("base"), RigidTransform([0, 0, 0.7])
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

meshcat_vis = builder.AddSystem(
    MeshcatVisualizer(scene_graph, zmq_url="new", open_browser=False)
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
balance_controller = builder.AddSystem(BalanceController(plant))

builder.Connect(plant.get_state_output_port(), balance_controller.get_input_port(0))
builder.Connect(balance_controller.get_output_port(), plant.get_actuation_input_port())
builder.Connect(
    scene_graph.get_query_output_port(), meshcat_vis.get_geometry_query_input_port()
)
builder.Connect(
    plant.GetOutputPort("contact_results"), contact_viz.GetInputPort("contact_results")
)
diagram = builder.Build()

# Physical-based Simulation
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyContextFromRoot(context)

set_home(plant, plant_context)
state_projection = np.zeros((24, 37))
state_projection[:12, 7:19] = np.eye(12)
state_projection[12:, 25:] = np.eye(12)
x0 = state_projection @ plant.get_state_output_port().Eval(plant_context)
balance_controller.get_input_port(1).FixValue(
    balance_controller.GetMyContextFromRoot(context), x0
)

# print("x0 = ", x0)
# print(context)

meshcat_vis.reset_recording()
meshcat_vis.start_recording()
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(8.0)
meshcat_vis.stop_recording()
meshcat_vis.publish_recording()

# contact_viz_context = diagram.GetMutableSubsystemContext(contact_viz, context)
# contact_results = contact_viz.EvalAbstractInput(
#     contact_viz_context, contact_input_port.get_index()
# ).get_value()
# for i_contact in range(contact_results.num_point_pair_contacts()):
#     contact_info = contact_results.point_pair_contact_info(i_contact)
#     print(contact_info.contact_force())
#     print(contact_info.contact_point())
#     print(contact_info.bodyA_index())
#     print(contact_info.bodyB_index())
#     print("===")

# print("************")
# contact_results = plant.get_contact_results_output_port().Eval(plant_context)
# for i_contact in range(contact_results.num_point_pair_contacts()):
#     contact_info = contact_results.point_pair_contact_info(i_contact)
#     print(contact_info.contact_force())
#     print(contact_info.contact_point())
#     print(contact_info.bodyA_index())
#     print(contact_info.bodyB_index())
#     print("===")


# plt.figure()
# plot_system_graphviz(diagram, max_depth=2)
# plt.show()

print("Done.")
# import time

# time.sleep(1e3)
