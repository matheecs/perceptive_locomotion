# -*-coding:UTF-8 -*-
import argparse
import timeit
from math import sqrt
from math import sinh
from math import cosh
import numpy as np
import matplotlib.pyplot as plt


class LIPM:
    def __init__(self, x0=0.0, v0=0.64, z0=0.6, obstacles_list=[]) -> None:
        self.x0 = x0
        self.v0 = v0

        self.xt = 0.0
        self.vt = 0.0
        self.step_at_world = 0.0

        # constant
        self.zt = z0
        self.g = 9.8
        self.Tc = sqrt(self.zt / self.g)
        self.step_resolution = 0.02
        self.obstacles_list = obstacles_list
        self.obstacle_width = 0.03

    def update(self, x0, v0, t):
        tau = t / self.Tc
        xt = x0 * cosh(tau) + self.Tc * v0 * sinh(tau)
        vt = (x0 / self.Tc) * sinh(tau) + v0 * cosh(tau)
        return xt, vt

    def getCapturePoint(self, xt, vt):
        capture_point = xt + np.sign(vt) * sqrt((vt ** 2) * self.zt / self.g)
        return capture_point

    def checkStepSafety(self, step_location_world):
        index = int(step_location_world / self.obstacle_width)
        if index in self.obstacles_list:
            return False
        else:
            return True

    def showStep(self, result_list):  # [[l1,l2,l3],[x1_0,x2_0,x3_0]]
        assert len(result_list) == 2

        # draw COM and foot
        plt.plot(
            [0],
            [self.zt],
            marker="o",
            markersize=20,
            color="r",
        )
        for i in range(0, 3):
            plt.plot(
                [
                    result_list[0][i] + result_list[1][i],
                ],
                [self.zt],
                marker="o",
                markersize=20,
                color="r",
            )
        plt.plot(
            [0, result_list[0][0], result_list[0][1], result_list[0][2]],
            [0, 0, 0, 0],
            "bo",
        )

        # draw legs
        plt.plot([0, 0], [0, self.zt], "k")
        plt.plot([0, result_list[0][0] + result_list[1][0]], [0, self.zt], "k")

        plt.plot(
            [result_list[0][0], result_list[0][0] + result_list[1][0]],
            [0, self.zt],
            "k",
        )
        plt.plot(
            [result_list[0][0], result_list[0][1] + result_list[1][1]],
            [0, self.zt],
            "k",
        )

        plt.plot(
            [result_list[0][1], result_list[0][1] + result_list[1][1]],
            [0, self.zt],
            "k",
        )
        plt.plot(
            [result_list[0][1], result_list[0][2] + result_list[1][2]],
            [0, self.zt],
            "k",
        )

        plt.plot(
            [result_list[0][2], result_list[0][2] + result_list[1][2]],
            [0, self.zt],
            "k",
        )

        # draw obstacles
        for i in self.obstacles_list:
            plt.plot(
                [i * self.obstacle_width, (i + 1 - 0.1) * self.obstacle_width],
                [0, 0],
                "r",
                linewidth=4.0,
            )

        plt.gca().set_aspect("equal", adjustable="box")
        plt.xlim([-0.2, 1.5])
        plt.ylim([-0.2, 0.8])
        plt.show()


def getCost(
    step_path_list,
    step_reference_path_list,
    capture_point_world_list,
    stable_weight=0.5,
):
    assert len(step_reference_path_list) == 3
    assert len(step_path_list) == 3
    assert len(capture_point_world_list) == 3

    foot_cost = 0
    stable_score = 0
    for i in range(0, 3):
        foot_cost = foot_cost + (step_path_list[i] - step_reference_path_list[i]) ** 2
        stable_score = (
            stable_score + (step_path_list[i] - capture_point_world_list[i]) ** 2
        )
    return foot_cost * (1 - stable_weight) + stable_score * stable_weight


if __name__ == "__main__":
    CLI = argparse.ArgumentParser()
    CLI.add_argument(
        "--list",  # name on the CLI - drop the `--` for positional/required parameters
        nargs="*",  # 0 or more values expected => creates a list
        type=int,
        default=[1, 2, 3],  # default if nothing is provided
    )
    input = CLI.parse_args()
    print("obstacles_list:", input.list)

    model = LIPM(obstacles_list=input.list)

    t_half_sample_list = [i / 30 * 1.0 * 0.5 for i in range(9, 13)]
    t_full_sample_list = [i / 30 * 1.0 for i in range(9, 13)]

    step_length_reference = 0.25
    l1_step_world_list = [
        i * model.step_resolution + step_length_reference for i in range(-5, 6)
    ]
    l2_step_world_list = [
        i * model.step_resolution + step_length_reference * 2 for i in range(-5, 6)
    ]
    l3_step_world_list = [
        i * model.step_resolution + step_length_reference * 3 for i in range(-5, 6)
    ]

    # print(t_half_sample_list)
    # print(t_full_sample_list)
    # print(l1_step_world_list)
    # print(l2_step_world_list)
    # print(l3_step_world_list)

    step_world_reference_list = [i * step_length_reference for i in range(1, 4)]

    result_list = [[0, 0, 0], [0, 0, 0]]  # [[l1,l2,l3],[x1_0,x2_0,x3_0]]
    mini_cost = float("inf")

    start = timeit.default_timer()
    for t0 in t_half_sample_list:
        capture_point_world_list = [0, 0, 0]

        capture_point_world_list[0] = model.getCapturePoint(model.x0, model.v0) + 0.0
        (x0f, v0f) = model.update(model.x0, model.v0, t0)

        for l1 in l1_step_world_list:
            # Step 1
            # Step 1
            # Step 1
            x1_0 = (x0f + 0) - l1
            if x1_0 > 0:
                continue
            if model.checkStepSafety(l1):
                for t1 in t_full_sample_list:
                    capture_point_world_list[1] = model.getCapturePoint(x1_0, v0f) + l1
                    (x1f, v1f) = model.update(x1_0, v0f, t1)

                    for l2 in l2_step_world_list:
                        # Step 2
                        # Step 2
                        # Step 2
                        x2_0 = (x1f + l1) - l2
                        if x2_0 > 0:
                            continue
                        if model.checkStepSafety(l2):
                            for t2 in t_full_sample_list:
                                capture_point_world_list[2] = (
                                    model.getCapturePoint(x2_0, v1f) + l2
                                )
                                (x2f, v2f) = model.update(x2_0, v1f, t2)

                                for l3 in l3_step_world_list:
                                    # Step 3
                                    # Step 3
                                    # Step 3
                                    x3_0 = (x2f + l2) - l3
                                    if x3_0 > 0:
                                        continue
                                    if model.checkStepSafety(l3):
                                        cost = getCost(
                                            [l1, l2, l3],
                                            step_world_reference_list,
                                            capture_point_world_list,
                                            0.5,
                                        )
                                        if cost < mini_cost:
                                            mini_cost = cost
                                            result_list[0] = [l1, l2, l3]
                                            result_list[1] = [x1_0, x2_0, x3_0]
                                    else:
                                        continue
                        else:
                            continue
            else:
                continue

    stop = timeit.default_timer()
    print("Time:", stop - start)

    if mini_cost < float("inf"):
        model.showStep(result_list)
    else:
        print("No step path!")
