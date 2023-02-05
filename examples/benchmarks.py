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

T_START = 5
T_END = 100
T_STEP = 40
NUM_PARALLEL = 3


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
    axes[0].set_xlabel('run time [s]', fontsize=14)
    axes[0].set_ylabel('solution cost', fontsize=14)
    axes[0].legend()
    axes[1].set_xlabel('run time [s]', fontsize=14)
    axes[1].set_ylabel('success [%]', fontsize=14)
    axes[1].legend()
    fig.tight_layout()

    # plt.title(name, fontsize=16, fontname='DejaVu Serif', loc='center', y=2, pad=3)
    plt.subplots_adjust(hspace=0.3)
    plt.show()


class Benchmark:
    def __init__(self, env, num_parallel):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)
        self.env = env
        self.robot = self.env.robot
        self.num_parallel = num_parallel

        pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                           self.env.poobjects_properties,
                                           self.robot.cam_link_id, self.robot.cam_orientation,
                                           self.env.goal_states, self.env.space_name, self.env.bounds, 0, 0)
        self.belief_states = pb_ompl_interface.si.getWorld().getAllBeliefStates()
        self.world = pb_ompl_interface.si.getWorld()

        self.res = []
        for i in range(num_parallel):
            self.res.append([])
            for _ in range(len(self.belief_states)):
                self.res[i].append([])
        self.success = []
        for i in range(num_parallel):
            self.success.append([])

        self.res_avg = []
        for _ in range(len(self.belief_states)):
            self.res_avg.append([])

        self.res_final = []
        self.success_avg = []

    def plan(self, planning_time, seed, sampler):
        costs = [0] * len(self.belief_states)
        costs_simplified = [0] * len(self.belief_states)

        pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                                self.env.poobjects_properties,
                                                self.robot.cam_link_id, self.robot.cam_orientation,
                                                self.env.goal_states, self.env.space_name, self.env.bounds,
                                                planning_time, 0)

        pb_ompl_interface.set_obstacles(self.env.obstacles)
        pb_ompl_interface.set_planner("Partial")
        if sampler != "default":
            pb_ompl_interface.set_state_sampler_name(sampler, seed)
        pb_ompl_interface.ss.getProblemDefinition().setSeed(seed)

        self.robot.set_state(self.env.start)
        res, paths, _ = pb_ompl_interface.plan(self.env.goal)

        for i, path in enumerate(paths):
            idx = pb_ompl_interface.ss.getProblemDefinition().getSolutionIdx()[i]
            costs[idx] = calc_cost(path)
            costs_simplified[idx] = calc_cost(pb_ompl_interface.ss.getProblemDefinition().getRawSolutions()[i])

        for i in range(len(self.res[seed-1])):
            if costs[i] > 0:
                self.res[seed-1][i].append([planning_time, costs[i], costs_simplified[i]])
        self.success[seed-1].append([planning_time, res])

    def plan_dummy(self, planning_time, seed, sampler):
        for i in range(len(self.res[seed-1])):
            self.res[seed-1][i].append([planning_time, random.randint(0,9), random.randint(0,9)])
        self.success[seed-1].append([planning_time, random.randint(0,1)])

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

        self.res_avg = np.mean(self.res, axis=0)
        self.success_avg.append(np.mean(self.success, axis=0))

        res_ = []
        for i, r in enumerate(self.res_avg):
            if i == 0:
                res_ = r
            else:
                r[:, 0] = 0
                res_ = np.add(res_, r * self.world.getBeliefStateProbability(i))
        self.res_final.append(res_)

    def reset(self):
        self.res = []
        for i in range(self.num_parallel):
            self.res.append([])
            for _ in range(len(self.belief_states)):
                self.res[i].append([])
        self.success = []
        for i in range(self.num_parallel):
            self.success.append([])

        self.res_avg = []
        for _ in range(len(self.belief_states)):
            self.res_avg.append([])

    def create_graph(self, name, save):
        fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
        axes[0].plot([c[0] for c in self.res_final[0]], [c[1] for c in self.res_final[0]], label="default")
        axes[1].plot([c[0] for c in self.success_avg[0]], [c[1] for c in self.success_avg[0]], label="default")
        axes[0].plot([c[0] for c in self.res_final[1]], [c[1] for c in self.res_final[1]], label="camera")
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
        axes[0].set_xlabel('run time [s]', fontsize=14)
        axes[0].set_ylabel('solution cost', fontsize=14)
        # axes[0].set_title(name, fontsize=16)
        axes[0].legend()
        axes[1].set_xlabel('run time [s]', fontsize=14)
        axes[1].set_ylabel('success [%]', fontsize=14)
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
        env = environments.RoombaEnv()
        b = Benchmark(env, NUM_PARALLEL)
        devnull = open('/dev/null', 'w')
        oldstdout_fno = os.dup(sys.stdout.fileno())
        os.dup2(devnull.fileno(), 1)
        b.benchmark(T_START, T_END, T_STEP, "default")
        b.reset()
        b.benchmark(T_START, T_END, T_STEP, "camera")
        b.create_graph("roomba_simple_" + str(T_START) + "-" + str(T_END) + "-" + str(T_STEP) + "-" + str(NUM_PARALLEL), True)
    else:
        load_graph("test")
