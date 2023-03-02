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
    def __init__(self, env, termination_iterations, interpolation_num, seed=-1, sampler="camera", init_belief=[]):
        super().__init__(env, 0, termination_iterations, interpolation_num, seed, sampler, init_belief)

    def draw_robot(self, pos="start", idx=0, x="0", y="0"):
        if pos == "start":
            self.robot.set_state(self.env.start)
        elif pos == "goal":
            self.robot.set_state(self.env.goal)
        elif pos == "pogoal":
            self.robot.set_state(self.env.goal_states[idx])
        elif pos == "obs":
            s = self.pb_ompl_interface.ss.getProblemDefinition().getObservationPointStates()[idx]
            state = self.pb_ompl_interface.state_to_list(s)
            self.robot.set_state(state)
        else:
            pass

    def add_robot(self):
        for i in range(p.getNumJoints(self.robot.id)):
            color = p.getVisualShapeData(self.robot.id, i)[0][7]
            new_color = list(color[:-1]) + [1]
            p.changeVisualShape(self.robot.id, i, rgbaColor=new_color)
        color = p.getVisualShapeData(self.robot.id, -1)[0][7]
        new_color = list(color[:-1]) + [1]
        p.changeVisualShape(self.robot.id, -1, rgbaColor=new_color)

    def remove_robot(self):
        for i in range(p.getNumJoints(self.robot.id)):
            color = p.getVisualShapeData(self.robot.id, i)[0][7]
            new_color = list(color[:-1]) + [0]
            p.changeVisualShape(self.robot.id, i, rgbaColor=new_color)
        color = p.getVisualShapeData(self.robot.id, -1)[0][7]
        new_color = list(color[:-1]) + [0]
        p.changeVisualShape(self.robot.id, -1, rgbaColor=new_color)

    def add_plane(self):
        p.loadURDF("../models/floor/floor.urdf", useFixedBase=True, basePosition=(0, 0, -0.01))

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

    def draw_paths_single(self):
        self.plan()
        demo.add_plane()
        demo.remove_robot()
        demo.draw_start([0, 0, 0, 1])
        p_worlds = self.pb_ompl_interface.ss.getProblemDefinition().getPWorlds()
        path_points = -1
        path_idx = 0
        path_param = p.addUserDebugParameter("Path", 0, len(self.paths) - 1, 0)
        observation_points = []
        while True:
            if path_idx != int(p.readUserDebugParameter(path_param)):
                path_idx = int(p.readUserDebugParameter(path_param))
                for i in observation_points:
                    p.removeBody(i)
                self.pb_ompl_interface.update_poobjects_probability(p_worlds[path_idx])

                p.removeUserDebugItem(path_points)
                points = [[point[0], point[1], 0] for point in self.paths[path_idx]]
                point_colors = [[0, 0, 0] for _ in self.paths[path_idx]]
                path_points = p.addUserDebugPoints(points, point_colors, 5, 0)
                observation_points = self.draw_path_observation_point(path_idx)

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

    def draw_path_observation_point(self, path_idx):
        po_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0.5, 0, 0.5, 1]]
        added_ids = []
        already_added = dict()
        for i, state in enumerate(self.pb_ompl_interface.ss.getProblemDefinition().getObservationPointStates()):
            observations = []
            for observation in self.pb_ompl_interface.ss.getProblemDefinition().getObservationPointObservations()[i]:
                observations.append(observation)
            s = self.pb_ompl_interface.state_to_list(state)
            key_state = str(s[0]) + "," + str(s[1])
            if not (key_state in already_added):
                if s in self.paths[path_idx]:
                    added_ids.append(self.pb_ompl_interface.add_debug_point([s[0], s[1]], 0.2, po_colors[observations[0]]))
                    already_added[key_state] = 1
            else:
                if s in self.paths[path_idx]:
                    added_ids.append(self.pb_ompl_interface.add_debug_point([s[0], s[1]], 0.2, po_colors[observations[0]]))
                    already_added[key_state] += 1
        return added_ids


if __name__ == '__main__':
    # time.sleep(10)
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # env = environments.RoombaDoorEnv()
    # demo = EnvImg(env, 2000, 1000, seed=42, sampler="camera")
    # demo.draw_paths_parallel(False)
    # demo.draw_observation_points()
    # demo.draw_goal([0, 0, 0, 1])
    # demo.add_plane()
    # demo.draw_robot(pos="start")

    # env = environments.SearchAndRescueSimpleEnv()
    # demo = EnvImg(env, 2000, 1000, seed=42, sampler="camera")
    # demo.draw_paths_parallel(True)
    # demo.draw_observation_points()
    # demo.add_plane()
    # demo.draw_robot(pos="start")

    # env = environments.SearchAndRescueSimpleEnv()
    # demo = EnvImg(env, 2000, 1000, seed=42, sampler="camera")
    # demo.draw_paths_parallel(True)
    # demo.draw_observation_points()
    # demo.draw_start([0, 0, 0, 1])
    # demo.add_plane()
    # demo.draw_robot(pos="pogoal", idx=0)

    # env = environments.SearchAndRescueSimpleEnv()
    # demo = EnvImg(env, 2000, 1000, seed=42, sampler="camera", init_belief=[0.33, 0.33, 0.33, 0.01])
    # demo.draw_paths_parallel(True)
    # demo.draw_observation_points()
    # demo.add_plane()
    # demo.draw_robot(pos="start")

    # env = environments.MobileArmObservationPointEnv()
    # demo = EnvImg(env, 2000, 1000, seed=5, sampler="camera")
    # demo.draw_paths_parallel(True)
    # demo.draw_observation_points()
    # demo.draw_start([0, 0, 0, 1])
    # demo.add_plane()
    # demo.draw_robot(pos="obs", idx=0)

    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    # env = environments.ParkingCornerEnv()
    # demo = EnvImg(env, 2000, 1000, seed=42, sampler="camera")
    # demo.draw_paths_single()

    env = environments.RoombaEnv()
    demo = EnvImg(env, 1000, 1000, seed=42, sampler="camera")
    demo.draw_paths_parallel(False)
    demo.draw_goal([0, 0, 0, 1])
    demo.add_plane()
    # demo.draw_robot(pos="start")
    demo.remove_robot()
    demo.draw_start([0, 0, 0, 1])
    demo.draw_observation_points()


    # demo.draw_robot(pos="pogoal", idx=0)
    # demo.draw_robot(pos="obs", idx=0)
    visibility = 1
    param = p.addUserDebugParameter('Robot visibility', 0, 1, 1)
    while True:
        if visibility != int(p.readUserDebugParameter(param)):
            visibility = int(p.readUserDebugParameter(param))
            if visibility == 0:
                demo.remove_robot()
            else:
                demo.add_robot()
