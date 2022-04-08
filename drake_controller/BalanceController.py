import numpy as np
from pydrake.all import LeafSystem, BasicVector


class BalanceController(LeafSystem):
    """
    Methods:
      0s-2s use PD controller
      2s-8s use QP controller
    q_v_estimated_state(37):
      qw,qx,qy,qz,body_x,body_y,body_z,joint_angle_{fl.hx->hr.kn} INDEX  0->18
      body_spatial_velocity,joint_angular_rate_{fl.hx->hr.kn}     INDEX 19->36
    """

    def __init__(self, plant):
        LeafSystem.__init__(self)
        self.plant = plant
        print("This is [BalanceController]")

        self.input_index_qv_estimated_states = self.DeclareVectorInputPort(
            "q_v_estimated_state", BasicVector(plant.get_state_output_port().size())
        ).get_index()
        print(self.input_index_qv_estimated_states)

        self.input_index_desired_state = self.DeclareVectorInputPort(
            "desired_joint_state", BasicVector(24)
        ).get_index()
        print(self.input_index_desired_state)

        self.output_index_tau = self.DeclareVectorOutputPort(
            "tau",
            BasicVector(plant.get_actuation_input_port().size()),
            self.calcTorqueOutput,
        )
        print(self.output_index_tau)

    def calcTorqueOutput(self, context, output):
        if context.get_time() < 2.0:
            q_v = self.GetInputPort("q_v_estimated_state").Eval(context)
            joint_q_v_des = self.GetInputPort("desired_joint_state").Eval(context)
            kp = 200
            kd = 50
            state_projection = np.zeros((24, 37))
            state_projection[:12, 7:19] = np.eye(12)
            state_projection[12:, 25:] = np.eye(12)
            joint_q_v = state_projection @ q_v

            # print("q_v:", q_v)
            # print("joint_q_v:", joint_q_v)

            pd = kp * (joint_q_v_des[:12] - joint_q_v[:12]) + kd * (
                joint_q_v_des[12:24] - joint_q_v[12:24]
            )

            output_projection = self.plant.MakeActuationMatrix()[6:, :].T
            output.SetFromVector(output_projection @ pd)
        else:
            output.SetFromVector(np.zeros(12))
