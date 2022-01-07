# -*-coding:UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from LIPM_2D import LIPM_2D

# functions definition for animation
def initAnimation():
    LIPM_left_leg_ani.set_data([], [])
    LIPM_right_leg_ani.set_data([], [])
    LIPM_COM_ani.set_data(COM_x[0], COM_z[0])
    COM_pos.set_text("")

    xt_point_ani.set_data(0, xt_array[0])
    vt_point_ani.set_data(0, vt_array[0])
    xvt_point_ani.set_data(xt_array[0], vt_array[0])

    return [
        LIPM_left_leg_ani,
        LIPM_right_leg_ani,
        LIPM_COM_ani,
        COM_pos,
        xt_point_ani,
        vt_point_ani,
        xvt_point_ani,
    ]


def animate(index):
    left_leg_x = [COM_x[index], left_foot_x_array[index]]
    left_leg_y = [COM_z[index], left_foot_z_array[index]]
    right_leg_x = [COM_x[index], right_foot_x_array[index]]
    right_leg_y = [COM_z[index], right_foot_z_array[index]]

    if left_foot_support_state[index]:
        LIPM_left_leg_ani.set_data(left_leg_x, left_leg_y)
        LIPM_right_leg_ani.set_data([], [])
    else:
        LIPM_left_leg_ani.set_data([], [])
        LIPM_right_leg_ani.set_data(right_leg_x, right_leg_y)
    LIPM_COM_ani.set_data(COM_x[index], COM_z[index])
    COM_pos.set_text(COM_str % (COM_x[index], COM_z[index]))

    xt_point_ani.set_data(index, xt_array[index])
    vt_point_ani.set_data(index, vt_array[index])
    xvt_point_ani.set_data(xt_array[index], vt_array[index])

    return [
        LIPM_left_leg_ani,
        LIPM_right_leg_ani,
        LIPM_COM_ani,
        COM_pos,
        xt_point_ani,
        vt_point_ani,
        xvt_point_ani,
    ]


x0 = 0
v0 = 0.64
z0 = 0.6
delta_t = 0.02
LIPM_model = LIPM_2D(x0, v0, z0, delta_t)
LIPM_model.target_orbital_energy = 0.2

data_len = 100
switch_index = 18

orbital_energy_array = list()
xt_array = list()
vt_array = list()
COM_x = list()
COM_z = list()
left_foot_x_array = list()
left_foot_z_array = list()
right_foot_x_array = list()
right_foot_z_array = list()
left_foot_support_state = list()

j = 0
swing_foot_reference_x = [0]
for i in range(int(switch_index / 2) + 1, data_len + 1):
    LIPM_model.update()

    if i <= switch_index:  # first step
        LIPM_model.capture_point = LIPM_model.updateCapturePoint(
            LIPM_model.xt, LIPM_model.vt, LIPM_model.target_orbital_energy
        )
        if LIPM_model.support_leg == "left_leg":
            LIPM_model.right_foot_x = LIPM_model.left_foot_x + LIPM_model.capture_point
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.left_foot_x)
            COM_z.append(LIPM_model.zt)
        else:
            LIPM_model.left_foot_x = LIPM_model.right_foot_x + LIPM_model.capture_point
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.right_foot_x)
            COM_z.append(LIPM_model.zt)
    else:  # continue walking after the first step
        if LIPM_model.support_leg == "left_leg":
            LIPM_model.right_foot_x = swing_foot_reference_x[j]
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.left_foot_x)
            COM_z.append(LIPM_model.zt)
        else:
            LIPM_model.left_foot_x = swing_foot_reference_x[j]
            # record the COM position
            COM_x.append(LIPM_model.xt + LIPM_model.right_foot_x)
            COM_z.append(LIPM_model.zt)
        j = j + 1  # update the index for reference swing foot position

    # switch the support leg and calculate necessary information
    if (i >= switch_index) and (i % switch_index == 0):
        LIPM_model.switchSupportLeg()

        # calculate the capture point at the time of the next support leg switch
        xt, vt = LIPM_model.updateXtVt(
            LIPM_model.x0, LIPM_model.v0, LIPM_model.delta_t * (switch_index)
        )
        orbital_energy = LIPM_model.updateOrbitalEnergy(xt, vt)
        capture_point = LIPM_model.updateCapturePoint(
            xt, vt, LIPM_model.target_orbital_energy
        )

        # interpolation the reference foot trajectory of the swing leg from current position to the next capture point
        if LIPM_model.support_leg == "left_leg":
            swing_foot_reference_x = np.linspace(
                LIPM_model.right_foot_x,
                LIPM_model.left_foot_x + capture_point,
                switch_index,
            )
        else:
            swing_foot_reference_x = np.linspace(
                LIPM_model.left_foot_x,
                LIPM_model.right_foot_x + capture_point,
                switch_index,
            )
        j = 0

    # record the data
    xt_array.append(LIPM_model.xt)
    vt_array.append(LIPM_model.vt)
    left_foot_x_array.append(LIPM_model.left_foot_x)
    left_foot_z_array.append(LIPM_model.left_foot_z)
    right_foot_x_array.append(LIPM_model.right_foot_x)
    right_foot_z_array.append(LIPM_model.right_foot_z)
    orbital_energy_array.append(LIPM_model.orbital_energy)
    if LIPM_model.support_leg == "left_leg":
        left_foot_support_state.append(True)
    else:
        left_foot_support_state.append(False)


# plot the animation
fig = plt.figure(figsize=(12, 6))

ax = fig.add_subplot(411, autoscale_on=False, xlim=(-0.5, 2), ylim=(-0.1, 0.8))
ax.grid(ls="--")
(LIPM_left_leg_ani,) = ax.plot([], [], "o-", lw=4, color="b")
(LIPM_right_leg_ani,) = ax.plot([], [], "o-", lw=4, color="k")
(LIPM_COM_ani,) = ax.plot([], [], marker="o", markersize=20, color="r")
COM_str = "COM = (%.1f, %.1f)"
COM_pos = ax.text(0.05, 0.9, "", transform=ax.transAxes)

bx = fig.add_subplot(412, autoscale_on=True)
bx.grid(ls="--")
bx.plot(xt_array)
(xt_point_ani,) = bx.plot([], [], marker="o", color="r")
plt.xlabel("time (s)")
plt.ylabel("COM position x")

cx = fig.add_subplot(413, autoscale_on=True)
cx.grid(ls="--")
cx.plot(vt_array)
(vt_point_ani,) = cx.plot([], [], marker="o", color="r")
plt.xlabel("time (s)")
plt.ylabel("COM velocity")

dx = fig.add_subplot(414, autoscale_on=True)
dx.grid(ls="--")
dx.plot(xt_array, vt_array)
(xvt_point_ani,) = dx.plot([], [], marker="o", color="r")
plt.xlabel("x")
plt.ylabel("v")


ani_LIPM = animation.FuncAnimation(
    fig=fig,
    init_func=initAnimation,
    func=animate,
    frames=range(1, data_len - int(switch_index / 2)),
    interval=1.0 / delta_t,
    blit=True,
)

ani_LIPM.save("step_path_planning.gif", writer="imagemagick")
plt.show()
