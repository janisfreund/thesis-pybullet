import math
import os
import os.path as osp
import time

import pybullet as p
import sys
import matplotlib.pyplot as plt
import pickle
import progressbar
import numpy as np
import csv

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import environments
import demos

ITERATIONS_START = 80
ITERATIONS_END = 120
ITERATIONS_STEP = 10
NUM_PARALLEL = 2


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


def load_graph_from_pickle(name):
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


def load_graph_from_data(name):
    path = "./benchmark_data/" + name
    costs_cam = []
    with open(path + "/costs_camera.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            costs_cam.append([float(value) for value in row])
    costs_def = []
    with open(path + "/costs_default.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            costs_def.append([float(value) for value in row])
    suc_cam = []
    with open(path + "/success_camera.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            suc_cam.append([float(value) for value in row])
    suc_def = []
    with open(path + "/success_default.csv", "r") as file:
        reader = csv.reader(file)
        for row in reader:
            suc_def.append([float(value) for value in row])

    fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
    axes[0].plot([c[0] for c in costs_def], [c[2] for c in costs_def], label="default")
    axes[1].plot([c[0] for c in suc_def], [c[1] for c in suc_def], label="default")
    axes[0].plot([c[0] for c in costs_cam], [c[2] for c in costs_cam], label="camera")
    axes[1].plot([c[0] for c in suc_cam], [c[1] for c in suc_cam], label="camera")

    axes[0].set_xlabel('iterations', fontsize=14)
    axes[0].set_ylabel('solution cost', fontsize=14)
    axes[0].set_xlim(ITERATIONS_START, ITERATIONS_END)
    axes[0].legend()
    axes[1].set_xlabel('iterations', fontsize=14)
    axes[1].set_ylabel('success [%]', fontsize=14)
    axes[1].set_xlim(ITERATIONS_START, ITERATIONS_END)
    axes[1].legend()

    fig.tight_layout()
    plt.subplots_adjust(hspace=0.3)
    plt.show()


class Benchmark:
    def __init__(self, num_parallel, env):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

        self.num_parallel = num_parallel
        self.env = env

        self.res = []
        for i in range(num_parallel):
            self.res.append([])
        self.success = []
        for i in range(num_parallel):
            self.success.append([])

        self.res_avg = []
        self.success_avg = []

        path = "./benchmark_data/wip"
        try:
            os.mkdir(path)
        except OSError:
            pass
        files = [f for f in os.listdir(path)]
        for f in files:
            os.remove(os.path.join(path, f))

    def plan(self, env, planning_time, seed, sampler, idx):
        demo = demos.Demo(env, 0, planning_time, 1000, seed=seed, sampler=sampler)
        demo.plan()

        path = "./benchmark_data/wip"
        if demo.costs > 0:
            self.res[seed-1].append([planning_time, 0, demo.costs])
            with open(path + "/costs_" + str(idx) + ".csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([seed, planning_time, 0, demo.costs])
        else:
            self.res[seed-1].append([np.nan, np.nan, np.nan])
            with open(path + "/costs_" + str(idx) + ".csv", "a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([seed, np.nan, np.nan, np.nan])
        self.success[seed-1].append([planning_time, demo.res])
        with open(path + "/success_" + str(idx) + ".csv", "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([seed, planning_time, demo.res])


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
        idx = 0
        if sampler == "camera":
            idx = 1

        for t in range(max_time, min_tme - 1, -time_interval):
            for seed in range(1, self.num_parallel + 1):
                # self.plan(t, seed, sampler)
                self.plan(self.env, t, seed, sampler, idx)
                t_elapsed += t
                bar.update(int(t_elapsed * multiplier))
            sampler = "stored"

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

    def save_data(self, name):
        path = "./benchmark_data/" + name
        try:
            os.mkdir(path)
        except OSError:
            pass
        files = [f for f in os.listdir(path)]
        for f in files:
            os.remove(os.path.join(path, f))
        with open(path + "/costs_default.csv", "w", newline="") as file:
            writer = csv.writer(file)
            for row in self.res_avg[0]:
                writer.writerow(row)
        with open(path + "/costs_camera.csv", "w", newline="") as file:
            writer = csv.writer(file)
            for row in self.res_avg[1]:
                writer.writerow(row)
        with open(path + "/success_default.csv", "w", newline="") as file:
            writer = csv.writer(file)
            for row in self.success_avg[0]:
                writer.writerow(row)
        with open(path + "/success_camera.csv", "w", newline="") as file:
            writer = csv.writer(file)
            for row in self.success_avg[1]:
                writer.writerow(row)

    def create_graph(self, name, save):
        fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
        axes[0].plot([c[0] for c in self.res_avg[0]], [c[2] for c in self.res_avg[0]], label="default")
        axes[1].plot([c[0] for c in self.success_avg[0]], [c[1] for c in self.success_avg[0]], label="default")
        axes[0].plot([c[0] for c in self.res_avg[1]], [c[2] for c in self.res_avg[1]], label="camera")
        axes[1].plot([c[0] for c in self.success_avg[1]], [c[1] for c in self.success_avg[1]], label="camera")

        if save:
            path = "./benchmark_data/" + name
            self.save_data(name)
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
        fig.savefig(path + "/" + name + ".png")
        plt.show()


if __name__ == '__main__':
    if True:
        p.connect(p.GUI)
        env = environments.RoombaEnv()
        b = Benchmark(NUM_PARALLEL, env)

        devnull = open('/dev/null', 'w')
        oldstdout_fno = os.dup(sys.stdout.fileno())
        os.dup2(devnull.fileno(), 1)

        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "default")
        b.reset()
        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "camera")
        b.create_graph("roomba_door_" + str(ITERATIONS_START) + "-" + str(ITERATIONS_END) + "-" + str(ITERATIONS_STEP) + "-" + str(NUM_PARALLEL), True)
    elif True:
        load_graph_from_data("roomba_simple_80-120-40-2")
    else:
        load_graph_from_pickle("roomba_simple_80-120-40-2")
