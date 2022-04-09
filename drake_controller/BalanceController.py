import numpy as np
from pydrake.all import (
    LeafSystem,
    BasicVector,
    EventStatus,
    Quaternion,
    RotationMatrix,
    MathematicalProgram,
    Solve,
)
from manifpy import SO3


def normalize(x):
    return x / np.linalg.norm(x)


def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


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
            period_sec=1.0, offset_sec=0, update=self._on_periodic_discrete
        )

    def _on_periodic_discrete(self, context, discrete_state):
        q_v_estimated = self.GetInputPort("q_v_estimated_state").Eval(context)
        print("At time:", context.get_time(), "s, q_v_estimated:\n", q_v_estimated)
        p_com = q_v_estimated[4:7]
        pd_com = q_v_estimated[22:25]
        R = RotationMatrix(Quaternion(wxyz=normalize(q_v_estimated[0:4])))
        w = q_v_estimated[19:22]
        print("p_com", p_com, "\npd_com", pd_com, "\nR", R, "\nw", w)
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

            # measurements in world
            p_com = q_v_estimated[4:7]
            pd_com = q_v_estimated[22:25]
            quaternion = Quaternion(wxyz=normalize(q_v_estimated[0:4]))
            R_SO3 = SO3(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w())
            w = q_v_estimated[19:22]

            # desired states in world
            p_com_des = np.array([-0.03819818, -0.00066073, 0.63747422])
            pd_com_des = np.array([0, 0, 0])
            quaternion = Quaternion(wxyz=normalize([1, 0, 0, 0]))
            R_SO3_des = SO3(
                quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()
            )
            w_des = np.array([0, 0, 0])

            R_error_SO3 = R_SO3_des * R_SO3.inverse()

            # equation (2)
            Kp_p = 50.0 * np.eye(3)
            Kd_p = 10.0 * np.eye(3)
            Kp_w = 100.0 * np.eye(3)
            Kd_w = 10.0 * np.eye(3)
            pdd_com_des = Kp_p @ (p_com_des - p_com) + Kd_p @ (pd_com_des - pd_com)
            wd_des = Kp_w @ (R_error_SO3.log().coeffs()) + Kd_w @ (w_des - w)

            # equation (3)
            I_b = np.array([[0.0973333, 0, 0], [0, 1.02467, 0], [0, 0, 1.04493]])
            R_wb = RotationMatrix(Quaternion(wxyz=normalize(q_v_estimated[0:4])))
            R_bw = R_wb.transpose()
            I_w = (R_wb.multiply(I_b)) @ (R_bw.multiply(np.eye(3)))
            mass = 30
            g = 9.81
            b_des = np.concatenate((mass * (pdd_com_des + [0, 0, g]), I_w @ wd_des))

            # QP
            p_fl_foot = np.array([0.32254366, 0.16987541, 0.0])
            p_hr_foot = np.array([-0.27303829, -0.16202879, 0.0])
            self.F_prev = np.array([0, 0, mass * g / 2, 0, 0, mass * g / 2])

            # equation (4)
            mu = 1.0
            prog = MathematicalProgram()
            F = prog.NewContinuousVariables(6, "Forces")
            A = np.zeros((6, 6))
            A[0:3, 0:3] = np.eye(3)
            A[0:3, 3:6] = np.eye(3)
            A[3:6, 0:3] = skew(p_fl_foot - p_com)
            A[3:6, 3:6] = skew(p_hr_foot - p_com)
            alpha = 1.0
            beta = 1.0
            S = np.eye(6)
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
            prog.AddBoundingBoxConstraint(0, 2 * mass * g, F[2])
            prog.AddBoundingBoxConstraint(0, 2 * mass * g, F[5])

            prog.SetInitialGuess(F, self.F_prev)

            result = Solve(prog)
            print(f"optimal solution F in world: {result.GetSolution(F)}")
            self.F_prev = result.GetSolution(F)

            #  equation (4)

            output.SetFromVector(output_projection @ pd)
            return
