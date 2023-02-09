import math
import os
import os.path as osp
import pybullet as p
import sys
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
print(fm.get_font_names())
import pickle
import progressbar
import numpy as np
import random

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import environments

ITERATIONS_START = 120
ITERATIONS_END = 140
ITERATIONS_STEP = 20
NUM_PARALLEL = 1


def calc_cost(path):
    path_len = 0
    for i in range(len(path) - 1):
        dist = 0
        for n in range(2):
            diff = path[i][n] - path[i + 1][n]
            dist += diff * diff
        path_len += math.sqrt(dist)
    return path_len


def vector_to_string(vec):
    s = "["
    for v in vec:
        s += str(v)
        s += ", "
    s = s[0:-2]
    s += "]"
    return s


def load_graph(name):
    path = "./benchmark_data/" + name
    files = [f for f in os.listdir(path)]
    fig = (pickle.load(open(os.path.join(path, files[0]), "rb")))
    axes = fig.get_axes()
    axes[0].set_xlabel('iterations', fontsize=14)
    axes[0].set_ylabel('solution cost', fontsize=14)
    axes[0].set_xlim(ITERATIONS_START, ITERATIONS_END)
    axes[0].legend()
    axes[1].set_xlabel('iterations', fontsize=14)
    axes[1].set_ylabel('success [%]', fontsize=14)
    axes[1].set_xlim(ITERATIONS_START, ITERATIONS_END)
    axes[1].legend()
    fig.tight_layout()

    # plt.title(name, fontsize=16, fontname='DejaVu Serif', loc='center', y=2, pad=3)
    plt.subplots_adjust(hspace=0.3)
    plt.show()


class Benchmark:
    def __init__(self, num_parallel):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

        self.num_parallel = num_parallel

        self.res = []
        for i in range(num_parallel):
            self.res.append([])
        self.success = []
        for i in range(num_parallel):
            self.success.append([])

        self.res_avg = []
        self.success_avg = []

    def plan(self, planning_time, seed, sampler):
        env = environments.RoombaEnv()
        robot = env.robot

        costs = 0
        costs_non_simplified = 0

        pb_ompl_interface = pb_ompl.PbOMPL(env.robot, env.obstacles, env.poobjects,
                                                env.poobjects_properties,
                                                robot.cam_link_id, robot.cam_orientation,
                                                env.goal_states, env.space_name, env.bounds,
                                                planning_time, 0)

        pb_ompl_interface.set_obstacles(env.obstacles)
        pb_ompl_interface.set_planner("Partial")
        if sampler != "default":
            pb_ompl_interface.set_state_sampler_name(sampler, seed)
        pb_ompl_interface.ss.getProblemDefinition().setSeed(seed)
        pb_ompl_interface.ss.getProblemDefinition().setIterations(planning_time)

        robot.set_state(env.start)
        res, paths, _ = pb_ompl_interface.plan(env.goal)

        print("\nPath costs:")
        for i, path in enumerate(paths):
            belief_idx = pb_ompl_interface.ss.getProblemDefinition().getSolutionIdx()[i]
            prob = pb_ompl_interface.si.getWorld().getBeliefStateProbability(belief_idx)
            costs += calc_cost(path) * prob
            costs_non_simplified += calc_cost(pb_ompl_interface.ss.getProblemDefinition().getRawSolutions()[i]) * prob
            print(str(calc_cost(pb_ompl_interface.ss.getProblemDefinition().getRawSolutions()[i])) + " * " + str(prob))
        costs_planner = pb_ompl_interface.ss.getProblemDefinition().getSolutionCost()
        if costs > 0:
            self.res[seed-1].append([planning_time, costs, costs_planner])
            print("Costs: " + str(costs_non_simplified))
        else:
            self.res[seed-1].append([np.nan, np.nan, np.nan])
        self.success[seed-1].append([planning_time, res])

        del pb_ompl_interface

    def benchmark(self, min_tme, max_time, time_interval, sampler):
        widgets = [' [',
                   progressbar.Timer(format='elapsed time: %(elapsed)s'),
                   '] ',
                   progressbar.Bar('*'), ' (',
                   progressbar.ETA(), ') ',
                   ]

        bar = progressbar.ProgressBar(max_value=1000,
                                      widgets=widgets).start()

        t_total = 0
        for t in range(min_tme, max_time + 1, time_interval):
            t_total += (t * self.num_parallel)
        multiplier = 1000 / t_total

        t_elapsed = 0
        for t in range(min_tme, max_time + 1, time_interval):
            for seed in range(1, self.num_parallel + 1):
                # self.plan(t, seed, sampler)
                self.plan(t, seed, sampler)
                t_elapsed += t
                bar.update(int(t_elapsed * multiplier))

        self.success_avg.append(np.mean(self.success, axis=0))
        del_idx = []
        for i in range(len(self.res[0])):
            all_none = True
            for n in range(len(self.res)):
                if not np.isnan(self.res[n][i][0]):
                    all_none = False
                    break
            if all_none:
                del_idx.append(i)
        self.res = np.delete(self.res, del_idx, axis=1)

        if len(self.res[0]) > 0:
            self.res_avg.append(np.nanmean(self.res, axis=0))
        else:
            self.res_avg.append([[0, 0, 0]])


    def reset(self):
        self.res = []
        for i in range(self.num_parallel):
            self.res.append([])
        self.success = []
        for i in range(self.num_parallel):
            self.success.append([])

    def create_graph(self, name, save):
        fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
        axes[0].plot([c[0] for c in self.res_avg[0]], [c[2] for c in self.res_avg[0]], label="default")
        axes[1].plot([c[0] for c in self.success_avg[0]], [c[1] for c in self.success_avg[0]], label="default")
        axes[0].plot([c[0] for c in self.res_avg[1]], [c[2] for c in self.res_avg[1]], label="camera")
        axes[1].plot([c[0] for c in self.success_avg[1]], [c[1] for c in self.success_avg[1]], label="camera")

        if save:
            path = "./benchmark_data/" + name
            try:
                os.mkdir(path)
            except OSError:
                pass
            files = [f for f in os.listdir(path)]
            for f in files:
                os.remove(os.path.join(path, f))
            pickle.dump(fig, open(path + "/" + name + ".pickle", "wb"))
        axes[0].set_xlabel('iterations', fontsize=14)
        axes[0].set_ylabel('solution cost', fontsize=14)
        axes[0].set_xlim(ITERATIONS_START, ITERATIONS_END)
        # axes[0].set_title(name, fontsize=16)
        axes[0].legend()
        axes[1].set_xlabel('iterations', fontsize=14)
        axes[1].set_ylabel('success [%]', fontsize=14)
        axes[1].set_xlim(ITERATIONS_START, ITERATIONS_END)
        # axes[1].set_title(name, fontsize=16)
        axes[1].legend()
        fig.tight_layout()

        # plt.title(name, fontsize=16, fontname='DejaVu Serif', loc='center', y=2, pad=3)
        plt.subplots_adjust(hspace=0.3)
        plt.savefig(path + "/" + name + ".png")
        plt.show()


if __name__ == '__main__':
    if True:
        p.connect(p.GUI)

        b = Benchmark(NUM_PARALLEL)

        # devnull = open('/dev/null', 'w')
        # oldstdout_fno = os.dup(sys.stdout.fileno())
        # os.dup2(devnull.fileno(), 1)

        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "default")
        b.reset()
        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "camera")
        b.create_graph("roomba_simple_" + str(ITERATIONS_START) + "-" + str(ITERATIONS_END) + "-" + str(ITERATIONS_STEP) + "-" + str(NUM_PARALLEL), True)
    else:
        load_graph("test")
