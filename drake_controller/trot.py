import numpy as np
from pydrake.all import (
    LeafSystem,
    BasicVector,
    EventStatus,
    Quaternion,
    RollPitchYaw,
    RotationMatrix,
    MathematicalProgram,
    Solve,
)
from manifpy import SO3
from math import sin, cos


def normalize(x):
    return x / np.linalg.norm(x)


def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


link_1 = 0.1108
link_2 = 0.32
link_3 = 0.34
x_offset_hip = 0.29785
y_offset_hip = 0.055
z_offset_hip = 0


def fl_forward_kinematics(q):
    offset = np.array([x_offset_hip, y_offset_hip, z_offset_hip])
    links = np.array([link_1, -link_2, -link_3])
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    t1 = q[0]
    t2 = q[1]
    t3 = q[2]
    foot_position = np.array([0.0, 0.0, 0.0])
    foot_position[0] = l2 * sin(t2) + l3 * sin(t2 + t3) + offset[0]
    foot_position[1] = (
        l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3) + offset[1]
    )
    foot_position[2] = (
        l1 * sin(t1) + l2 * cos(t1) * cos(t2) + l3 * cos(t1) * cos(t2 + t3) + offset[2]
    )
    return foot_position


def fl_jacobian(q):
    links = np.array([link_1, -link_2, -link_3])
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    t1 = q[0]
    t2 = q[1]
    t3 = q[2]
    jac = np.zeros((3, 3), dtype=float)
    jac[0, 0] = 0.0
    jac[0, 1] = l2 * cos(t2) + l3 * cos(t2 + t3)
    jac[0, 2] = l3 * cos(t2 + t3)
    jac[1, 0] = -l1 * sin(t1) - l2 * cos(t1) * cos(t2) - l3 * cos(t1) * cos(t2 + t3)
    jac[1, 1] = (l2 * sin(t2) + l3 * sin(t2 + t3)) * sin(t1)
    jac[1, 2] = l3 * sin(t1) * sin(t2 + t3)
    jac[2, 0] = l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3)
    jac[2, 1] = -(l2 * sin(t2) + l3 * sin(t2 + t3)) * cos(t1)
    jac[2, 2] = -l3 * sin(t2 + t3) * cos(t1)
    return jac


def hr_forward_kinematics(q):
    offset = np.array([-x_offset_hip, -y_offset_hip, z_offset_hip])
    links = np.array([-link_1, -link_2, -link_3])
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    t1 = q[0]
    t2 = q[1]
    t3 = q[2]
    foot_position = np.array([0.0, 0.0, 0.0])
    foot_position[0] = l2 * sin(t2) + l3 * sin(t2 + t3) + offset[0]
    foot_position[1] = (
        l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3) + offset[1]
    )
    foot_position[2] = (
        l1 * sin(t1) + l2 * cos(t1) * cos(t2) + l3 * cos(t1) * cos(t2 + t3) + offset[2]
    )
    return foot_position


def hr_jacobian(q):
    links = np.array([-link_1, -link_2, -link_3])
    l1 = links[0]
    l2 = links[1]
    l3 = links[2]
    t1 = q[0]
    t2 = q[1]
    t3 = q[2]
    jac = np.zeros((3, 3), dtype=float)
    jac[0, 0] = 0.0
    jac[0, 1] = l2 * cos(t2) + l3 * cos(t2 + t3)
    jac[0, 2] = l3 * cos(t2 + t3)
    jac[1, 0] = -l1 * sin(t1) - l2 * cos(t1) * cos(t2) - l3 * cos(t1) * cos(t2 + t3)
    jac[1, 1] = (l2 * sin(t2) + l3 * sin(t2 + t3)) * sin(t1)
    jac[1, 2] = l3 * sin(t1) * sin(t2 + t3)
    jac[2, 0] = l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3)
    jac[2, 1] = -(l2 * sin(t2) + l3 * sin(t2 + t3)) * cos(t1)
    jac[2, 2] = -l3 * sin(t2 + t3) * cos(t1)
    return jac


class BalanceController(LeafSystem):
    """
    Methods:
      0s-2s use PD controller
      2s-5s use QP controller
    q_v_estimated_state(37):
      qw,qx,qy,qz,body_x,body_y,body_z,joint_angle_{fl.hx->hr.kn} INDEX [0:19)
      body_spatial_velocity,joint_angular_rate_{fl.hx->hr.kn}     INDEX [19:37)
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
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.1, offset_sec=0, update=self._on_periodic_discrete
        )

        mass = 16.0
        g = 9.81
        self.F_prev = np.array([0, 0, mass * g / 2, 0, 0, mass * g / 2])

    def _on_periodic_discrete(self, context, discrete_state):
        q_v_estimated = self.GetInputPort("q_v_estimated_state").Eval(context)
        print("At time:", context.get_time())
        p_com = q_v_estimated[4:7]
        pd_com = q_v_estimated[22:25]
        R = RotationMatrix(Quaternion(wxyz=normalize(q_v_estimated[0:4])))
        RPY = RollPitchYaw(R)
        w = q_v_estimated[19:22]
        print("p_com", p_com, "\npd_com", pd_com, "\nRPY", RPY.vector(), "\nw", w)
        print("====================================================")
        self.called_periodic_discrete = True
        return EventStatus.Succeeded()

    def calcTorqueOutput(self, context, output):
        if context.get_time() <= 2.0:
            """
            Keep Standing Use PD
            """
            q_v_estimated = self.GetInputPort("q_v_estimated_state").Eval(context)
            joint_q_v_des = self.GetInputPort("desired_joint_state").Eval(context)
            kp = 200
            kd = 50
            state_projection = np.zeros((24, 37))
            state_projection[:12, 7:19] = np.eye(12)
            state_projection[12:, 25:] = np.eye(12)
            joint_q_v = state_projection @ q_v_estimated
            pd = kp * (joint_q_v_des[:12] - joint_q_v[:12]) + kd * (
                joint_q_v_des[12:24] - joint_q_v[12:24]
            )
            output_projection = self.plant.MakeActuationMatrix()[6:, :].T
            output.SetFromVector(output_projection @ pd)
            return
        else:
            """
            Force Control
            References:
                Bledt, Gerardo, et al. "MIT Cheetah 3: Design and control of a robust, dynamic quadruped robot." 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.
            output:
                fr(3->6) & hl(6->9) use PD
                fl(0->3) & hr(9->12) use QP
            """
            q_v_estimated = self.GetInputPort("q_v_estimated_state").Eval(context)
            joint_q_v_des = self.GetInputPort("desired_joint_state").Eval(context)
            joint_q_v_des[3:6] = np.array([0.0, 1.0, -2.0])
            joint_q_v_des[6:9] = np.array([0.0, 1.0, -2.0])
            kp = 50
            kd = 10
            state_projection = np.zeros((24, 37))
            state_projection[:12, 7:19] = np.eye(12)
            state_projection[12:, 25:] = np.eye(12)
            joint_q_v = state_projection @ q_v_estimated
            pd = kp * (joint_q_v_des[:12] - joint_q_v[:12]) + kd * (
                joint_q_v_des[12:24] - joint_q_v[12:24]
            )
            output_projection = self.plant.MakeActuationMatrix()[6:, :].T

            # measurements in world
            p_com = q_v_estimated[4:7]
            pd_com = q_v_estimated[22:25]
            quaternion = Quaternion(wxyz=normalize(q_v_estimated[0:4]))
            R_SO3 = SO3(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w())
            w = q_v_estimated[19:22]

            # desired states in world
            p_com_des = np.array([-0.03819818, -0.00066073, 0.63747422])
            pd_com_des = np.array([0.0, 0.0, 0.0])
            quaternion_des = Quaternion(wxyz=normalize([1, 0, 0, 0]))

            R_SO3_des = SO3(
                quaternion_des.x(),
                quaternion_des.y(),
                quaternion_des.z(),
                quaternion_des.w(),
            )
            w_des = np.array([0.0, 0.0, 0.0])

            R_error_SO3 = R_SO3_des * R_SO3.inverse()

            # equation (2)
            Kp_p = 50.0 * np.eye(3)
            Kd_p = 10.0 * np.eye(3)
            Kp_w = 50000.0 * np.eye(3)
            Kd_w = 10.0 * np.eye(3)
            pdd_com_des = Kp_p @ (p_com_des - p_com) + Kd_p @ (pd_com_des - pd_com)

            # aa_m = RotationMatrix(quaternion).ToAngleAxis()
            # aa_d = RotationMatrix(quaternion_des).ToAngleAxis()

            wd_des = Kp_w @ (R_error_SO3.log().coeffs()) + Kd_w @ (w_des - w)

            # print("aa_d-aa_m", aa_d.axis() * aa_d.angle() - aa_m.axis() * aa_m.angle())
            # print("R_error_SO3.log().coeffs()", R_error_SO3.log().coeffs())

            print("p_com_des - p_com", p_com_des - p_com)
            print("pd_com_des - pd_com", pd_com_des - pd_com)
            print("R_error_SO3.log().coeffs()", R_error_SO3.log().coeffs())
            print("w_des - w", w_des - w)

            # equation (3)
            I_b = np.array([[0.0973333, 0, 0], [0, 1.02467, 0], [0, 0, 1.04493]])
            R_wb = RotationMatrix(Quaternion(wxyz=normalize(q_v_estimated[0:4])))
            R_bw = R_wb.transpose()
            I_w = (R_wb.multiply(I_b)) @ (R_bw.multiply(np.eye(3)))
            mass = 16.0
            g = 9.81
            b_des = np.concatenate((mass * (pdd_com_des + [0, 0, g]), I_w @ wd_des))

            # QP
            p_fl_foot = np.array([0.32254366, 0.16987541, 0.0])
            p_hr_foot = np.array([-0.27303829, -0.16202879, 0.0])
            # self.F_prev = np.array([0, 0, mass * g / 2, 0, 0, mass * g / 2])

            # equation (4)
            mu = 1.0
            prog = MathematicalProgram()
            F = prog.NewContinuousVariables(6, "Forces")
            A = np.zeros((6, 6))
            A[0:3, 0:3] = np.eye(3)
            A[0:3, 3:6] = np.eye(3)
            A[3:6, 0:3] = skew(p_fl_foot - p_com)
            A[3:6, 3:6] = skew(p_hr_foot - p_com)
            alpha = 100.0
            beta = 10.0
            S = 500 * np.eye(6)
            prog.AddQuadraticErrorCost(
                Q=A.T @ S @ A, x_desired=np.linalg.pinv(A) @ b_des, vars=F
            )
            # prog.AddCost((A @ F - b_des).T @ S @ (A @ F - b_des))
            prog.AddQuadraticCost(alpha * F.dot(F))
            F_delta = F - self.F_prev
            prog.AddQuadraticCost(beta * F_delta.dot(F_delta))
            prog.AddLinearConstraint(F[0] <= mu * F[2])
            prog.AddLinearConstraint(F[1] <= mu * F[2])
            prog.AddLinearConstraint(F[3] <= mu * F[5])
            prog.AddLinearConstraint(F[4] <= mu * F[5])
            prog.AddLinearConstraint(-F[0] <= mu * F[2])
            prog.AddLinearConstraint(-F[1] <= mu * F[2])
            prog.AddLinearConstraint(-F[3] <= mu * F[5])
            prog.AddLinearConstraint(-F[4] <= mu * F[5])
            prog.AddBoundingBoxConstraint(0.0, 2 * mass * g, F[2])
            prog.AddBoundingBoxConstraint(0.0, 2 * mass * g, F[5])

            prog.SetInitialGuess(F, self.F_prev)

            result = Solve(prog)
            print(f"optimal solution F in world: {result.GetSolution(F)}")
            self.F_prev = result.GetSolution(F)

            F_fl_w = self.F_prev[0:3]
            F_hr_w = self.F_prev[3:6]
            joint_q_fl = q_v_estimated[7:10]
            joint_q_hr = q_v_estimated[16:19]

            J_fl = fl_jacobian(joint_q_fl)
            J_hr = hr_jacobian(joint_q_hr)

            tau_fl = -J_fl.T @ R_bw.multiply(F_fl_w)
            tau_hr = -J_hr.T @ R_bw.multiply(F_hr_w)

            # print("tau_fl", tau_fl)
            # print("tau_hr", tau_hr)

            torque_output = output_projection @ pd
            torque_output[0:3] = tau_fl + 0.0 * torque_output[0:3]
            torque_output[9:12] = tau_hr + 0.0 * torque_output[9:12]
            print("torque_output", torque_output)

            # torque_output = np.array([0.0] * 12)
            output.SetFromVector(torque_output)

            return


# print(fl_forward_kinematics(np.array([0.0, 0.0, 0.0])))
# print(hr_forward_kinematics(np.array([0.0, 0.0, 0.0])))
# print(fl_jacobian(np.array([0.0, 0.0, 0.0])))
# print(hr_jacobian(np.array([0.0, 0.0, 0.0])))
