from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import numpy as np


# initial state
x0 = 0
v0 = 0.64

# some constant
z = 0.6
g = 9.8
Tc = np.sqrt(z / g)
full_stance_time = 10 / 30
half_stance_time = 0.5 * full_stance_time
full_tau = full_stance_time / Tc
half_tau = 0.5 * full_tau
reference_com_velocity = v0 * 0.7 / 0.6
reference_step_length = full_stance_time * (v0 * 0.7 / 0.6)

print("reference_step_length: ", reference_step_length)


def predict(x0, v0, t):
    tau = t / Tc
    pt = x0 * np.cosh(tau) + Tc * v0 * np.sinh(tau)
    vt = x0 / Tc * np.sinh(tau) + v0 * np.cosh(tau)
    return pt, vt


prog = MathematicalProgram()


# ADD decision variables
X = prog.NewContinuousVariables(3, "X")
V = prog.NewContinuousVariables(3, "V")
F = prog.NewContinuousVariables(3, "Foot")


# ADD constrains
p_init = x0 * np.cosh(half_tau) + Tc * v0 * np.sinh(half_tau)
v_init = x0 / Tc * np.sinh(half_tau) + v0 * np.cosh(half_tau)

# kinematics
prog.AddConstraint(X[0] <= 0)
prog.AddConstraint(X[1] <= 0)
prog.AddConstraint(X[2] <= 0)
prog.AddConstraint(X[0] >= -0.3)
prog.AddConstraint(X[1] >= -0.3)
prog.AddConstraint(X[2] >= -0.3)
prog.AddConstraint(F[1] - F[0] + X[1] <= 0.3)
prog.AddConstraint(F[2] - F[1] + X[2] <= 0.3)

prog.AddConstraint(
    lambda z: np.array([z[0] * np.cosh(full_tau) + Tc * z[1] * np.sinh(full_tau)]),
    lb=[0.1],
    ub=[reference_step_length],
    vars=[X[0], V[0]],
)
prog.AddConstraint(
    lambda z: np.array([z[0] * np.cosh(full_tau) + Tc * z[1] * np.sinh(full_tau)]),
    lb=[0.1],
    ub=[reference_step_length],
    vars=[X[1], V[1]],
)


prog.AddConstraint(
    lambda z: np.array([p_init - z[0] - z[1]]),
    lb=[0.0],
    ub=[0.0],
    vars=[X[0], F[0]],
)
prog.AddConstraint(
    lambda z: np.array([z[0] - v_init]),
    lb=[0.0],
    ub=[0.0],
    vars=[V[0]],
)


prog.AddConstraint(
    lambda z: np.array(
        [
            z[0] * np.cosh(full_tau)
            + Tc * z[1] * np.sinh(full_tau)
            - z[2]
            - (z[4] - z[3])
        ]
    ),
    lb=[0.0],
    ub=[0.0],
    vars=[X[0], V[0], X[1], F[0], F[1]],
)
prog.AddConstraint(
    lambda z: np.array(
        [z[0] / Tc * np.sinh(full_tau) + z[1] * np.cosh(full_tau) - z[2]]
    ),
    lb=[0.0],
    ub=[0.0],
    vars=[X[0], V[0], V[1]],
)


prog.AddConstraint(
    lambda z: np.array(
        [
            z[0] * np.cosh(full_tau)
            + Tc * z[1] * np.sinh(full_tau)
            - z[2]
            - (z[4] - z[3])
        ]
    ),
    lb=[0.0],
    ub=[0.0],
    vars=[X[1], V[1], X[2], F[1], F[2]],
)
prog.AddConstraint(
    lambda z: np.array(
        [z[0] / Tc * np.sinh(full_tau) + z[1] * np.cosh(full_tau) - z[2]]
    ),
    lb=[0.0],
    ub=[0.0],
    vars=[X[1], V[1], V[2]],
)

# step obstacle
prog.AddConstraint(F[1] <= 0.45)
prog.AddConstraint(F[1] >= 0.40)

# ADD cost

# obstacle areas margin
weight_margin = 5
prog.AddCost(weight_margin * (0.425 - F[1]) ** 2)

# close to reference step path
prog.AddCost(
    (F[0] - reference_step_length) ** 2
    + (F[1] - reference_step_length * 2) ** 2
    + (F[2] - reference_step_length * 3) ** 2
)
# close to reference com velocity
prog.AddCost(
    (V[0] - reference_com_velocity) ** 2
    + (V[1] - reference_com_velocity) ** 2
    + (V[2] - reference_com_velocity) ** 2
)
# capture point (stability criteria)
prog.AddCost(
    (F[0] - (x0 + np.sign(v0) * np.sqrt((v0 ** 2) * z / g))) ** 2
    + (F[1] - F[0] - (X[0] + V[0] * np.sqrt(z / g))) ** 2
    + (F[2] - F[1] - (X[1] + V[1] * np.sqrt(z / g))) ** 2
)

# Now solve the optimization problem.
result = Solve(prog)

# print out the result.
print("Success?", result.is_success())
print("X    =  ", result.GetSolution(X))
print("V    =  ", result.GetSolution(V))
print("F    =  ", result.GetSolution(F))
print("cost =  ", result.get_optimal_cost())
print("solver: ", result.get_solver_id().name())
