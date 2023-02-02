import math
import os
import os.path as osp
import pybullet as p
import sys
import matplotlib.pyplot as plt
import pickle

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import environments


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
    axs = []
    for f in files:
        axs.append(pickle.load(open(os.path.join(path, f), "rb")))
    plt.xlabel('run time [s]', fontsize=14)
    plt.ylabel('solution cost', fontsize=14)
    plt.title(name, fontsize=16)
    plt.legend()
    plt.tight_layout()
    plt.show()


class Benchmark:
    def __init__(self, env):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)
        self.env = env
        self.robot = self.env.robot

        pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                           self.env.poobjects_properties,
                                           self.robot.cam_link_id, self.robot.cam_orientation,
                                           self.env.goal_states, self.env.space_name, self.env.bounds, 0, 0)
        self.belief_states = pb_ompl_interface.si.getWorld().getAllBeliefStates()

        self.res = []
        for i in range(len(self.belief_states)):
            self.res.append([])

    def plan(self, planning_time, avg_num, sampler):
        costs = [0] * len(self.belief_states)
        costs_simplified = [0] * len(self.belief_states)

        for run in range(avg_num):
            pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                                    self.env.poobjects_properties,
                                                    self.robot.cam_link_id, self.robot.cam_orientation,
                                                    self.env.goal_states, self.env.space_name, self.env.bounds,
                                                    planning_time, 0)

            pb_ompl_interface.set_obstacles(self.env.obstacles)
            pb_ompl_interface.set_planner("Partial")
            if sampler != "default":
                pb_ompl_interface.set_state_sampler_name("camera")

            self.robot.set_state(self.env.start)
            res, paths, _ = pb_ompl_interface.plan(self.env.goal)

            for i, path in enumerate(paths):
                idx = pb_ompl_interface.ss.getProblemDefinition().getSolutionIdx()[i]
                costs[idx] = (costs[idx] * run + calc_cost(path)) / (run + 1)
                costs_simplified[idx] = (costs_simplified[idx] * run + calc_cost(
                    pb_ompl_interface.ss.getProblemDefinition().getRawSolutions()[i])) / (run + 1)

        for i in range(len(self.res)):
            if costs[i] > 0:
                self.res[i].append([planning_time, costs[i], costs_simplified[i]])

    def benchmark(self, min_tme, max_time, time_interval, avg_num, sampler):
        for t in range(min_tme, max_time + 1, time_interval):
            self.plan(t, avg_num, sampler)

    def create_graph(self, name, save):
        for i in range(len(self.res)):
            if len(self.res[i]) > 0:
                ax = plt.plot([c[0] for c in self.res[i]], [c[1] for c in self.res[i]], label=vector_to_string(self.belief_states[i]))
                if save:
                    path = "./benchmark_data/" + name
                    try:
                        os.mkdir(path)
                    except OSError:
                        pass
                    files = [f for f in os.listdir(path)]
                    for f in files:
                        os.remove(os.path.join(path, f))
                    pickle.dump(ax, open(path + "/" + name + "_" + str(i) + ".pickle", "wb"))

                # plt.plot([c[0] for c in self.res[i]], [c[2] for c in self.res[i]], label=vector_to_string(self.belief_states[i]))
        plt.xlabel('run time [s]', fontsize=14)
        plt.ylabel('solution cost', fontsize=14)
        plt.title(name, fontsize=16)
        plt.legend()
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    if False:
        p.connect(p.GUI)
        env = environments.RoombaEnv()
        b = Benchmark(env)
        b.benchmark(10, 20, 10, 1, "camera")
        b.create_graph("toy_example_10-20", True)
    else:
        load_graph("toy_example_10-20")
