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

import matplotlib.font_manager
# fpaths = matplotlib.font_manager.findSystemFonts()
# for i in fpaths:
#     try:
#         f = matplotlib.font_manager.get_font(i)
#         print(f.family_name)
#     except:
#         pass

import environments
import demos
"""
0: toy example
1: door env
2: search and rescue env
3: observation point env
4: parking env
"""
ENV = 2
CONTINUE = True
ITERATIONS_START = 0
ITERATIONS_END = 6000
ITERATIONS_STEP = 100
NUM_PARALLEL = 4


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

    plt.rcParams["font.family"] = 'Liberation Serif'
    fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
    axes[0].plot([c[0] for c in costs_def], [c[2] for c in costs_def], label="default", color="dodgerblue")
    axes[1].plot([c[0] for c in suc_def], [c[1] for c in suc_def], label="default",
                 color="dodgerblue")
    axes[0].plot([c[0] for c in costs_cam], [c[2] for c in costs_cam], label="camera", color="darkorange")
    axes[1].plot([c[0] for c in suc_cam], [c[1] for c in suc_cam], label="camera",
                 color="darkorange")

    axes[0].fill_between([c[0] for c in costs_def], [c[2] - c[3] for c in costs_def],
                         [c[2] + c[3] for c in costs_def], color="dodgerblue", alpha=0.3, linewidth=0)

    axes[0].fill_between([c[0] for c in costs_cam], [c[2] - c[3] for c in costs_cam],
                         [c[2] + c[3] for c in costs_cam], color="darkorange", alpha=0.3, linewidth=0)

    axes[0].set_xlabel('iterations', fontsize=14)
    axes[0].set_ylabel('solution cost', fontsize=14)
    axes[0].set_xlim(ITERATIONS_START, ITERATIONS_END)
    y_scale = max(np.max(np.array(costs_def)[:, 5]), np.max(np.array(costs_cam)[:, 5])) + 0.5
    axes[0].set_ylim(-0.05, y_scale)
    axes[0].legend(loc='upper left', bbox_to_anchor=(0., 1.3),
                   fancybox=True, shadow=True)
    axes[1].set_xlabel('iterations', fontsize=14)
    axes[1].set_ylabel('success [%]', fontsize=14)
    axes[1].set_xlim(ITERATIONS_START, ITERATIONS_END)
    axes[1].set_ylim(-0.05, 1.05)
    fig.tight_layout()
    plt.subplots_adjust(hspace=0.3)
    fig.savefig(path + "/" + name + ".png")
    plt.show()


class Benchmark:
    def __init__(self, num_parallel, env, continue_wip, name):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

        self.num_parallel = num_parallel
        self.env = env
        self.name = name

        self.res = []
        for i in range(num_parallel):
            self.res.append([])
        self.success = []
        for i in range(num_parallel):
            self.success.append([])

        self.res_avg = []
        self.success_avg = []

        self.continue_wip = continue_wip

        path = "./benchmark_data/wip_" + self.name + "_" + str(ITERATIONS_START) + "-" + str(ITERATIONS_END) + "-" + str(ITERATIONS_STEP)
        try:
            os.mkdir(path)
        except OSError:
            pass
        if not continue_wip:
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


    def benchmark_legacy(self, min_tme, max_time, time_interval, sampler):
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

    def benchmark(self, min_iterations, max_iterations, step_size, sampler):
        widgets = [' [',
                   progressbar.Timer(format='elapsed time: %(elapsed)s'),
                   '] ',
                   progressbar.Bar('*'), ' (',
                   progressbar.ETA(), ') ',
                   ]

        bar = progressbar.ProgressBar(max_value=self.num_parallel,
                                      widgets=widgets).start()

        path = "./benchmark_data/wip_" + self.name + "_" + str(ITERATIONS_START) + "-" + str(ITERATIONS_END) + "-" + str(ITERATIONS_STEP)
        wip = self.continue_wip
        if not os.path.exists(path + "/costs_" + sampler + ".csv"):
            wip = False

        max_seed_saved = 1
        if wip:
            costs_saved = []
            with open(path + "/costs_" + sampler + ".csv", "r") as file:
                reader = csv.reader(file)
                for row in reader:
                    costs_saved.append([float(value) for value in row])
            max_seed_saved = int(costs_saved[-1][0]) + 1

        for seed in range(max_seed_saved, self.num_parallel + 1):
            demo = demos.Demo(env, 0, max_iterations, 1000, seed=seed, sampler=sampler)
            demo.init_benchmark_mode(min_iterations, max_iterations, step_size)
            demo.plan()
            costs = demo.get_benchmark_results()
            for i, c in enumerate(costs):
                iteration = min_iterations + (i * step_size)
                if math.isinf(c):
                    self.res[seed-1].append([np.nan, np.nan, np.nan])
                    self.success[seed-1].append([iteration, 0])
                    with open(path + "/costs_" + sampler + ".csv", "a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([seed, np.nan, np.nan, np.nan])
                    with open(path + "/success_" + sampler + ".csv", "a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([seed, iteration, 0])
                else:
                    self.res[seed-1].append([iteration, 0, c])
                    self.success[seed - 1].append([iteration, 1])
                    with open(path + "/costs_" + sampler + ".csv", "a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([seed, iteration, 0, c])
                    with open(path + "/success_" + sampler + ".csv", "a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([seed, iteration, 1])
            bar.update(seed)

        if wip:
            self.reset()
            costs_saved = []
            with open(path + "/costs_" + sampler + ".csv", "r") as file:
                reader = csv.reader(file)
                for row in reader:
                    costs_saved.append([float(value) for value in row])
            max_seed_saved = costs_saved[-1][0]
            for s in range(0, int(max_seed_saved)):
                r = [c[1:] for c in costs_saved if c[0] == s+1]
                self.res[s] = r
            success_saved = []
            with open(path + "/success_" + sampler + ".csv", "r") as file:
                reader = csv.reader(file)
                for row in reader:
                    success_saved.append([float(value) for value in row])
            max_seed_saved = success_saved[-1][0]
            for s in range(0, int(max_seed_saved)):
                r = [c[1:] for c in success_saved if c[0] == s+1]
                self.success[s] = r

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
            mean = np.nanmean(self.res, axis=0)
            r = np.append(mean, np.transpose([np.nanstd(self.res, axis=0)[:,2]]), axis=1)
            r = np.append(r, np.transpose([np.nanmin(self.res, axis=0)[:,2]]), axis=1)
            r = np.append(r, np.transpose([np.nanmax(self.res, axis=0)[:,2]]), axis=1)
            self.res_avg.append(r)
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

    def create_graph(self, save):
        plt.rcParams["font.family"] = 'Liberation Serif'
        fig, axes = plt.subplots(2, 1, figsize=(8, 8), dpi=300)
        axes[0].plot([c[0] for c in self.res_avg[0]], [c[2] for c in self.res_avg[0]], label="default", color="dodgerblue")
        axes[1].plot([c[0] for c in self.success_avg[0]], [c[1] for c in self.success_avg[0]], label="default", color="dodgerblue")
        axes[0].plot([c[0] for c in self.res_avg[1]], [c[2] for c in self.res_avg[1]], label="camera", color="darkorange")
        axes[1].plot([c[0] for c in self.success_avg[1]], [c[1] for c in self.success_avg[1]], label="camera", color="darkorange")

        axes[0].fill_between([c[0] for c in self.res_avg[0]], [c[2] - c[3] for c in self.res_avg[0]],
                             [c[2] + c[3] for c in self.res_avg[0]], color="dodgerblue", alpha=0.3, linewidth=0)
        # axes[0].fill_between([c[0] for c in self.res_avg[0]], [c[4] for c in self.res_avg[0]],
        #                      [c[5] for c in self.res_avg[0]], color="dodgerblue", alpha=0.2, linewidth=0)

        axes[0].fill_between([c[0] for c in self.res_avg[1]], [c[2] - c[3] for c in self.res_avg[1]],
                             [c[2] + c[3] for c in self.res_avg[1]], color="darkorange", alpha=0.3, linewidth=0)
        # axes[0].fill_between([c[0] for c in self.res_avg[1]], [c[4] for c in self.res_avg[1]],
        #                      [c[5] for c in self.res_avg[1]], color="darkorange", alpha=0.2, linewidth=0)

        name = self.name + "_" + str(ITERATIONS_START) + "-" + str(ITERATIONS_END) + "-" + str(ITERATIONS_STEP) + "-" + str(NUM_PARALLEL)
        if save:
            path = "./benchmark_data/" + name
            self.save_data(name)
            pickle.dump(fig, open(path + "/" + name + ".pickle", "wb"))
        axes[0].set_xlabel('iterations', fontsize=14)
        axes[0].set_ylabel('solution cost', fontsize=14)
        axes[0].set_xlim(ITERATIONS_START, ITERATIONS_END)
        y_scale = max(np.max(self.res_avg[0][:, 5]), np.max(self.res_avg[1][:, 5])) + 0.5
        axes[0].set_ylim(-0.05, y_scale)
        axes[0].legend(loc='upper left', bbox_to_anchor=(0., 1.3),
          fancybox=True, shadow=True)
        axes[1].set_xlabel('iterations', fontsize=14)
        axes[1].set_ylabel('success [%]', fontsize=14)
        axes[1].set_xlim(ITERATIONS_START, ITERATIONS_END)
        axes[1].set_ylim(-0.05, 1.05)
        fig.tight_layout()
        plt.subplots_adjust(hspace=0.3)
        if save:
            fig.savefig(path + "/" + name + ".png")
        plt.show()


if __name__ == '__main__':
    if True:
        p.connect(p.GUI)
        if ENV == 0:
            env = environments.RoombaEnv()
            name = "toy"
        elif ENV == 1:
            env = environments.RoombaDoorEnv()
            name = "door"
        elif ENV == 2:
            env = environments.SearchAndRescueSimpleEnv()
            name = "sar"
        elif ENV == 3:
            env = environments.MobileArmObservationPointEnv()
            name = "obs"
        elif ENV == 4:
            env = environments.ParkingCornerEnv()
            name = "parking"
        b = Benchmark(NUM_PARALLEL, env, CONTINUE, name)

        devnull = open('/dev/null', 'w')
        oldstdout_fno = os.dup(sys.stdout.fileno())
        os.dup2(devnull.fileno(), 1)

        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "default")
        b.reset()
        b.benchmark(ITERATIONS_START, ITERATIONS_END, ITERATIONS_STEP, "camera")
        b.create_graph(True)
    else:
        load_graph_from_data("roomba_0-100-10-3")
