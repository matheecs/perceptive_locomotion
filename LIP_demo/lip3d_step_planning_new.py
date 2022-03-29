# %%

from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import numpy as np


# initial state
P_init = [0, 0]
V_init = [0.64, 0]
F_init = [0, 0]

# constants
Height = 0.6
Gravity = 9.8
Tc = np.sqrt(Height / Gravity)
Full_stance_time = 10 / 30
Half_stance_time = 0.5 * Full_stance_time
Reference_com_velocity = V_init[0] * 0.7 / 0.6
Reference_steps_length = Full_stance_time * (V_init[0] * 0.7 / 0.6)
print("Reference_com_velocity: ", Reference_com_velocity)
print("Reference_steps_length: ", Reference_steps_length)


# %%
import time

start_time = time.time()

prog = MathematicalProgram()
X = prog.NewContinuousVariables(32, "X")

# ADD Variables
# -----------P    V     A     F--
# index_0 = 0,1| 8, 9|16,17|24,25
# index_1 = 2,3|10,11|18,19|26,27
# index_2 = 4,5|12,13|20,21|28,29
# index_3 = 6,7|14,15|22,23|30,31

# P = prog.NewContinuousVariables(8, "P")  #  0-> 7
# V = prog.NewContinuousVariables(8, "V")  #  8->15
# A = prog.NewContinuousVariables(8, "A")  # 16->23
# F = prog.NewContinuousVariables(8, "F")  # 24->31

# ADD Constraints

# Dynamics constraint
def lip3d_dynamics(pi, vi, ai, time_left):
    tau = time_left / Tc
    p = [0, 0]
    v = [0, 0]
    for i in range(0, 2):
        p[i] = (
            pi[i] * np.cosh(tau)
            + Tc * vi[i] * np.sinh(tau)
            + 0.5 * ai[i] * time_left * time_left
        )
        v[i] = pi[i] / Tc * np.sinh(tau) + vi[i] * np.cosh(tau) + ai[i] * time_left
    return p, v


def constraint_Dynamics(z):
    [p_0f, v_0f] = lip3d_dynamics(
        [z[0], z[1]], [z[8], z[9]], [z[16], z[17]], Half_stance_time
    )
    [p_1f, v_1f] = lip3d_dynamics(
        [z[2], z[3]], [z[10], z[11]], [z[18], z[19]], Full_stance_time
    )
    [p_2f, v_2f] = lip3d_dynamics(
        [z[4], z[5]], [z[12], z[13]], [z[20], z[21]], Full_stance_time
    )

    return np.array(
        [
            # index 0 (initial state)
            z[0] - P_init[0],
            z[1] - P_init[1],
            z[8] - V_init[0],
            z[9] - V_init[1],
            # index 1
            (p_0f[0] - z[2]) - (z[26] - z[24]),
            (p_0f[1] - z[3]) - (z[27] - z[25]),
            v_0f[0] - z[10],
            v_0f[1] - z[11],
            # index 2
            (p_1f[0] - z[4]) - (z[28] - z[26]),
            (p_1f[1] - z[5]) - (z[29] - z[27]),
            v_1f[0] - z[12],
            v_1f[1] - z[13],
            # index 3
            (p_2f[0] - z[6]) - (z[30] - z[28]),
            (p_2f[1] - z[7]) - (z[31] - z[29]),
            v_2f[0] - z[14],
            v_2f[1] - z[15],
        ]
    )


prog.AddConstraint(
    constraint_Dynamics,
    lb=np.array([0.0] * 16),
    ub=np.array([0.0] * 16),
    vars=X,
)

# Acc constraint
prog.AddConstraint(
    lambda z: np.array([z[0], z[1], z[2], z[3], z[4], z[5], z[6], z[7]]),
    lb=np.array([0.0] * 8),
    ub=np.array([0.1] * 8),
    vars=X[16:24],
)
# %%

# Kinematics constraint (leg length)
def constraint_COP(z):
    [p_0f, v_0f] = lip3d_dynamics(
        [z[0], z[1]], [z[8], z[9]], [z[16], z[17]], Half_stance_time
    )
    [p_1f, v_1f] = lip3d_dynamics(
        [z[2], z[3]], [z[10], z[11]], [z[18], z[19]], Full_stance_time
    )
    [p_2f, v_2f] = lip3d_dynamics(
        [z[4], z[5]], [z[12], z[13]], [z[20], z[21]], Full_stance_time
    )
    return np.array(
        [
            # leg length
            p_0f[0] * p_0f[0] + p_0f[1] * p_0f[1],
            z[2] * z[2] + z[3] * z[3],
            p_1f[0] * p_1f[0] + p_1f[1] * p_1f[1],
            z[4] * z[4] + z[5] * z[5],
            p_2f[0] * p_2f[0] + p_2f[1] * p_2f[1],
            z[6] * z[6] + z[7] * z[7],
            # crossing the top (forward direction)
            p_0f[0],
            p_1f[0],
            p_2f[0],
            z[2],
            z[4],
            z[6],
        ]
    )


prog.AddConstraint(
    constraint_COP,
    lb=np.array([0.0 * 0.0] * 6 + [0.0] * 3 + [-0.4] * 3),
    ub=np.array([0.4 * 0.4] * 6 + [0.4] * 3 + [0.0] * 3),
    vars=X,
)

# %%

# Feet constraint (TODO)
prog.AddConstraint(
    lambda z: np.array([z[0], z[1], z[2], z[3], z[4], z[5]]),
    lb=[F_init[0], F_init[1], 0.2, -0.06, 0.42, 0.05],
    ub=[F_init[0], F_init[1], 0.3, -0.01, 0.48, 0.08],
    vars=X[24:],
)


# ADD COSTS

# Feet/Velocity Reference cost
weight = 1
prog.AddQuadraticErrorCost(
    Q=weight * np.identity(6),
    x_desired=np.array(
        [
            Reference_steps_length * 1,
            Reference_steps_length * 2,
            Reference_steps_length * 3,
            Reference_com_velocity,
            Reference_com_velocity,
            Reference_com_velocity,
        ]
    ),
    vars=[X[26], X[28], X[30], X[10], X[12], X[14]],
)
prog.AddQuadraticErrorCost(
    Q=np.identity(6),
    x_desired=np.array([0, 0, 0, 0, 0, 0]),
    vars=[X[27], X[29], X[31], X[11], X[13], X[15]],
)


# Capture point (stability criteria!!!sign->非凸函数) #TODO
def cost_Capture(z):
    f0_error = (z[26] - z[24]) - (P_init[0] + np.sign(V_init[0]) * V_init[0] * Tc)
    f1_error = (z[27] - z[25]) - (P_init[1] + np.sign(V_init[1]) * V_init[1] * Tc)
    f2_error = (z[28] - z[26]) - (z[2] + np.sign(z[10]) * z[10] * Tc)
    f3_error = (z[29] - z[27]) - (z[3] + np.sign(z[11]) * z[11] * Tc)
    f4_error = (z[30] - z[28]) - (z[4] + np.sign(z[12]) * z[12] * Tc)
    f5_error = (z[31] - z[29]) - (z[5] + np.sign(z[13]) * z[13] * Tc)

    return (
        f0_error ** 2
        + f1_error ** 2
        + f2_error ** 2
        + f3_error ** 2
        + f4_error ** 2
        + f5_error ** 2
    )


prog.AddCost(cost_Capture, vars=X)

print("--- Formulation %s seconds ---" % (time.time() - start_time))
start_time = time.time()
# %%
# Initialization

x_init = [0] * 32
x_init[0:8] = [
    0,
    0,
    Reference_steps_length * 0.5 * Full_stance_time,
    0,
    Reference_steps_length * 1.5 * Full_stance_time,
    0,
    Reference_steps_length * 2.5 * Full_stance_time,
    0,
]
x_init[8:16] = [Reference_com_velocity, 0] * 4
x_init[26:32] = [
    Reference_steps_length * 1,
    0,
    Reference_steps_length * 2,
    0,
    Reference_steps_length * 3,
    0,
]

prog.SetInitialGuess(X, x_init)

# Solve
result = Solve(prog)
print("--- Solving %s seconds ---" % (time.time() - start_time))

print("Success?", result.is_success())
# print("X    =\n", result.GetSolution(X))
# print("cost =  ", result.get_optimal_cost())
print("solver: ", result.get_solver_id().name())

X_sol = result.GetSolution(X)
# X_sol = np.array(X_sol)

# Plot the solution
import matplotlib.pyplot as plt

plt.figure()
for i in [0, 2, 4, 6]:
    plt.plot(X_sol[24 + i], X_sol[25 + i], "kH")
for i in [0, 2, 4, 6]:
    plt.plot(X_sol[0 + i] + X_sol[24 + i], X_sol[1 + i] + X_sol[25 + i], "bo")

for x, y in zip(X_sol[24:31:2], X_sol[25:32:2]):
    label = "footstep"
    plt.annotate(
        label,  # this is the text
        (x, y),  # these are the coordinates to position the label
        textcoords="offset points",  # how to position the text
        xytext=(0, 10),  # distance from text to points (x,y)
        ha="center",
    )  # horizontal alignment can be left, right or center

for x0, y0, x, y in zip(X_sol[0:7:2], X_sol[1:8:2], X_sol[24:31:2], X_sol[25:32:2]):
    label = "COM"
    plt.annotate(
        label,  # this is the text
        (x0 + x, y0 + y),  # these are the coordinates to position the label
        textcoords="offset points",  # how to position the text
        xytext=(0, 10),  # distance from text to points (x,y)
        ha="center",
    )  # horizontal alignment can be left, right or center

# Show feet areas
coord = [[0.2, 0.0], [0.3, 0.0], [0.3, -0.1], [0.2, -0.1]]
coord.append(coord[0])
xs, ys = zip(*coord)
plt.plot(xs, ys)


coord = [[0.4, 0.0], [0.5, 0.0], [0.5, 0.1], [0.4, 0.1]]
coord.append(coord[0])
xs, ys = zip(*coord)
plt.plot(xs, ys)


plt.xlabel("x Direction(forward)", size=12)
plt.ylabel("y Direction(left)", size=12)
plt.title("3D LIP Footstep Planning", size=15)
plt.axis("equal")
plt.show()
