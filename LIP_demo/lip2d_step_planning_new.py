from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import numpy as np


# initial state
x0 = 0
v0 = 0.64

# constants
z = 0.6
g = 9.8
Tc = np.sqrt(z / g)
full_stance_time = 10 / 30
half_stance_time = 0.5 * full_stance_time
reference_com_velocity = v0 * 0.7 / 0.6
reference_step_length = full_stance_time * (v0 * 0.7 / 0.6)
print("reference_step_length: ", reference_step_length)


def predict(x0, v0, t):
    tau = t / Tc
    xt = x0 * np.cosh(tau) + Tc * v0 * np.sinh(tau)
    vt = x0 / Tc * np.sinh(tau) + v0 * np.cosh(tau)
    return xt, vt


prog = MathematicalProgram()


# ADD Variables
X = prog.NewContinuousVariables(3, "X")
V = prog.NewContinuousVariables(3, "V")
F = prog.NewContinuousVariables(3, "F")

# ADD Constraints
# Dynamics constraint
[x_init, v_init] = predict(x0, v0, half_stance_time)


def constraint_Dynamics(z):  # z: X0 X1 X2 V0(3) V1 V2 F0(6) F1 F2
    [x_0f, v_0f] = predict(z[0], z[3], full_stance_time)
    [x_1f, v_1f] = predict(z[1], z[4], full_stance_time)
    return np.array(
        [
            (z[6] - 0) - (x_init - z[0]),
            z[3] - v_init,
            (z[7] - z[6]) - (x_0f - z[1]),
            z[4] - v_0f,
            (z[8] - z[7]) - (x_1f - z[2]),
            z[5] - v_1f,
        ]
    )


prog.AddConstraint(
    constraint_Dynamics,
    lb=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ub=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    vars=[X[0], X[1], X[2], V[0], V[1], V[2], F[0], F[1], F[2]],
)

# Kinematics constraint
def constraint_CoP(z):  # z: X0 X1 X2 F0 F1 F2
    return np.array([z[0], z[1], z[2], z[4] - z[3] + z[1], z[5] - z[4] + z[2]])


prog.AddConstraint(
    constraint_CoP,
    lb=np.array([-0.3, -0.3, -0.3, 0, 0]),
    ub=np.array([0.0, 0.0, 0.0, 0.3, 0.3]),
    vars=[X[0], X[1], X[2], F[0], F[1], F[2]],
)

# Feet constraint
prog.AddConstraint(lambda z: np.array([z[0]]), lb=[0.40], ub=[0.45], vars=[F[1]])


# ADD Costs
# Margin cost
weight_margin = 5
prog.AddCost(weight_margin * (0.425 - F[1]) ** 2)


# Step/Velocity Reference cost
prog.AddQuadraticErrorCost(
    Q=np.identity(6),
    x_desired=np.array(
        [
            reference_step_length,
            reference_step_length * 2,
            reference_step_length * 3,
            reference_com_velocity,
            reference_com_velocity,
            reference_com_velocity,
        ]
    ),
    vars=[F[0], F[1], F[2], V[0], V[1], V[2]],
)


# Capture point (stability criteria)
def cost_Capture(z):  # X0 X1 X2 V0(3) V1 V2 F0(6) F1 F2
    f0_error = (z[6] - 0) - (x0 + v0 * Tc)
    f1_error = (z[7] - z[6]) - (z[0] + z[3] * Tc)
    f2_error = (z[8] - z[7]) - (z[1] + z[4] * Tc)
    return f0_error ** 2 + f1_error ** 2 + f2_error ** 2


prog.AddCost(cost_Capture, vars=[X[0], X[1], X[2], V[0], V[1], V[2], F[0], F[1], F[2]])


# Solve
result = Solve(prog)
print("Success?", result.is_success())
print("X    =  ", result.GetSolution(X))
print("V    =  ", result.GetSolution(V))
print("F    =  ", result.GetSolution(F))
print("cost =  ", result.get_optimal_cost())
print("solver: ", result.get_solver_id().name())
