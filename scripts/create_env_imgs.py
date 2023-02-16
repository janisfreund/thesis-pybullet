import os.path as osp
import time
import pybullet as p
import sys

from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(abspath(__file__))), '../thesis-ompl/ompl/py-bindings'))
import environments

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import demos


def add_debug_point(pos, radius, color):
    visBoxId = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=0.01, rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, basePosition=[pos[0], pos[1], 0.005])


def path_to_list(path):
    l = []
    for i in range(len(path)):
        pl = []
        for n in range(len(path[i])):
            pl.append([path[i][n][0], path[i][n][1]])
        l.append(pl)
    return l


class EnvImg(demos.Demo):
    def __init__(self, env, termination_iterations, interpolation_num, seed=-1, sampler="camera"):
        super().__init__(env, 0, termination_iterations, interpolation_num, seed, sampler)
        # do following for all links
        color = p.getVisualShapeData(self.robot.id, -1)[0][7]
        new_color = list(color[:-1]) + [0]
        p.changeVisualShape(self.robot.id, -1, rgbaColor=new_color)
        for i in range(p.getNumJoints(self.robot.id)):
            color = p.getVisualShapeData(self.robot.id, i)[0][7]
            new_color = list(color[:-1]) + [0]
            p.changeVisualShape(self.robot.id, i, rgbaColor=new_color)

    def draw_robot(self, pos="start", x="0", y="0"):
        pass

    def draw_paths_parallel(self, belief_colors):
        self.plan()
        if belief_colors:
            colors = [[1,0,0], [0,1,0], [0,0,1], [0.5,0,0.5]]
        else:
            colors = [[0, 0, 0]]

        for i in range(self.pb_ompl_interface.ss.getProblemDefinition().getSolutionCount()):
            for n in range(len(self.pb_ompl_interface.tree_path_lists[i]) - 1):
                isSame = False
                for j in range(self.pb_ompl_interface.ss.getProblemDefinition().getSolutionCount()):
                    if i != j and len(self.pb_ompl_interface.tree_path_lists[j]) > n and self.pb_ompl_interface.tree_path_lists[j][n][0] == \
                            self.pb_ompl_interface.tree_path_lists[i][n][0] and self.pb_ompl_interface.tree_path_lists[j][n][1] == \
                            self.pb_ompl_interface.tree_path_lists[i][n][1] and self.pb_ompl_interface.tree_path_lists[j][n + 1][0] == \
                            self.pb_ompl_interface.tree_path_lists[i][n + 1][0] and self.pb_ompl_interface.tree_path_lists[j][n + 1][1] == \
                            self.pb_ompl_interface.tree_path_lists[i][n + 1][1]:
                        isSame = True
                if isSame:
                    p.addUserDebugLine([self.pb_ompl_interface.tree_path_lists[i][n][0], self.pb_ompl_interface.tree_path_lists[i][n][1], 0],
                                       [self.pb_ompl_interface.tree_path_lists[i][n + 1][0], self.pb_ompl_interface.tree_path_lists[i][n + 1][1], 0],
                                       lineColorRGB=[0, 0, 0], lineWidth=15)
                else:
                    p.addUserDebugLine([self.pb_ompl_interface.tree_path_lists[i][n][0], self.pb_ompl_interface.tree_path_lists[i][n][1], 0],
                                       [self.pb_ompl_interface.tree_path_lists[i][n + 1][0], self.pb_ompl_interface.tree_path_lists[i][n + 1][1],
                                        0], lineColorRGB=colors[self.pb_ompl_interface.belief_to_world(
                                        self.pb_ompl_interface.si.getWorld().getAllBeliefStates()[
                                        self.pb_ompl_interface.ss.getProblemDefinition().getSolutionIdx()[i]]) % len(colors)], lineWidth=5)

    def draw_observation_points(self):
        po_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0.5, 0, 0.5, 1]]
        already_added = dict()
        for i, state in enumerate(self.pb_ompl_interface.ss.getProblemDefinition().getObservationPointStates()):
            observations = []
            for observation in self.pb_ompl_interface.ss.getProblemDefinition().getObservationPointObservations()[i]:
                observations.append(observation)
            s = self.pb_ompl_interface.state_to_list(state)
            key_state = str(s[0]) + "," + str(s[1])
            if not (key_state in already_added):
                self.pb_ompl_interface.add_debug_point([s[0], s[1]], 0.1, po_colors[observations[0]])
                already_added[key_state] = 1
            else:
                self.pb_ompl_interface.add_debug_point([s[0], s[1]], 0.1, po_colors[observations[0]])
                already_added[key_state] += 1


if __name__ == '__main__':
    p.connect(p.GUI)
    env = environments.RoombaEnv()
    demo = EnvImg(env, 100, 1000, seed=1, sampler="camera")
    demo.draw_paths_parallel(False)
    demo.draw_observation_points()
    demo.draw_start([0, 0, 0, 1])
    demo.draw_goal([0, 0, 0, 1])

    input("Press Enter to continue...")
