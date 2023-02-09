import os.path as osp
import time
import pybullet as p
import sys
import math

from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(abspath(__file__))), '../thesis-ompl/ompl/py-bindings'))
from ompl import base as ob

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import robots as rb
import environments

DEMO_SELECTION = 0
"""
0: Roomba simple
1: Roomba doors <-
2: Roomba house
3: Mobile arm simple
4: Mobile arm hard <-
5: Mobile arm observation point <-
6: Search and rescue simple <-
7: Search and rescue office
8: Parking simple
9: Parking corner <-
"""


def add_debug_point(pos, radius, color):
    visBoxId = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=0.01, rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, basePosition=[pos[0], pos[1], 0.005])


def calc_cost(path):
    path_len = 0
    for i in range(len(path) - 1):
        dist = 0
        for n in range(2):
            diff = path[i][n] - path[i + 1][n]
            dist += diff * diff
        path_len += math.sqrt(dist)
    return path_len


def path_to_list(path):
    l = []
    for i in range(len(path)):
        pl = []
        for n in range(len(path[i])):
            pl.append([path[i][n][0], path[i][n][1]])
        l.append(pl)
    return l


class Demo:
    def __init__(self, env, termination_time, termination_iterations, interpolation_num, seed=-1):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)
        self.env = env
        self.robot = self.env.robot

        self.pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                                self.env.poobjects_properties,
                                                self.robot.cam_link_id, self.robot.cam_orientation,
                                                self.env.goal_states, self.env.space_name, self.env.bounds,
                                                termination_time, interpolation_num)

        self.pb_ompl_interface.set_obstacles(self.env.obstacles)
        self.pb_ompl_interface.set_planner("Partial")
        self.pb_ompl_interface.set_state_sampler_name("camera", seed)
        self.pb_ompl_interface.ss.getProblemDefinition().setSeed(seed)
        self.pb_ompl_interface.ss.getProblemDefinition().setIterations(termination_iterations)
        self.res = False
        self.paths = []

    def plan(self):
        self.robot.set_state(self.env.start)
        self.res, self.paths, _ = self.pb_ompl_interface.plan(self.env.goal)

    def draw_start(self, color):
        # draw start position
        add_debug_point([self.env.start[0], self.env.start[1], 0], 0.1, color)

    def draw_goal(self, color):
        # draw goal position
        add_debug_point([self.env.goal[0], self.env.goal[1], 0], 0.1, color)

    def print_costs(self):
        print("\nPath costs:")
        costs = 0
        for i, p in enumerate(self.paths):
            belief_idx = self.pb_ompl_interface.ss.getProblemDefinition().getSolutionIdx()[i]
            prob = self.pb_ompl_interface.si.getWorld().getBeliefStateProbability(belief_idx)
            non_simplified_path = self.pb_ompl_interface.ss.getProblemDefinition().getRawSolutions()[i]
            c = calc_cost(non_simplified_path)
            costs += c * prob
            print(str(c) + " * " + str(prob))
        print("Costs: " + str(costs))
        print("Costs given by planner: " + str(self.pb_ompl_interface.ss.getProblemDefinition().getSolutionCost()) + "\n")

    def demo_parallel(self, model, scale, RobotClass):
        if self.res:
            for poo in self.env.poobjects:
                p.setCollisionFilterGroupMask(poo, -1, 0, 0)
            robots = []
            for _ in self.paths:
                rid = p.loadURDF(model, (self.env.start[0], self.env.start[1], 0), globalScaling=scale)
                r = RobotClass(rid)
                robots.append(r)
            drawPath = True
            stepParam = ""
            raw_path_param = ""
            sol_line_ids = []
            line_id = []
            belief_colors = False
            if self.pb_ompl_interface.mode == 1:
                belief_colors = True
            while True:
                stepParam, raw_path_param, sol_line_ids, line_id = self.pb_ompl_interface.execute_all(self.paths,
                                                   drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                                                   linkid=self.robot.cam_link_id,
                                                   camera_orientation=self.robot.cam_orientation, robots=robots,
                                                   stepParam=stepParam, raw_path_param=raw_path_param,
                                                   sol_line_ids=sol_line_ids, line_id=line_id, belief_colors=belief_colors)

    def demo_consecutive(self):
        if self.res:
            for poo in self.env.poobjects:
                p.setCollisionFilterGroupMask(poo, -1, 0, 0)
            while True:
                self.pb_ompl_interface.execute_one_after_another(self.paths, True, camera=False,
                                                                 projectionMatrix=self.projectionMatrix,
                                                                 linkid=self.robot.cam_link_id,
                                                                 camera_orientation=self.robot.cam_orientation)


if __name__ == '__main__':
    # time.sleep(10)
    p.connect(p.GUI)

    if DEMO_SELECTION == 0:
        # simple roomba demo
        env = environments.RoombaEnv()
        demo = Demo(env, 0, 140, 1000, seed=1)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.draw_goal([0, 0, 0, 1])
        demo.print_costs()
        demo.demo_parallel("../models/create_description/urdf/create_2.urdf", 1, rb.Roomba)

    elif DEMO_SELECTION == 1:
        # simple door demo
        env = environments.RoombaDoorEnv()
        demo = Demo(env, 200, -1, 1000, seed=42)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.draw_goal([0, 0, 0, 1])
        demo.demo_parallel("../models/create_description/urdf/create_2.urdf", 1, rb.Roomba)

    elif DEMO_SELECTION == 2:
        # house roomba demo
        env = environments.RoombaHouseEnv()
        demo = Demo(env, 200, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.draw_goal([0, 0, 0, 1])
        demo.demo_parallel("../models/create_description/urdf/create_2.urdf", 1, rb.Roomba)

    elif DEMO_SELECTION == 3:
        # simple mobile arm demo
        env = environments.MobileArmEnv()
        demo = Demo(env, 20, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_parallel("../models/mobile_arm/mobile_arm.urdf", 1.25, rb.MobileArm)

    elif DEMO_SELECTION == 4:
        # mobile arm with walls demo
        env = environments.MobileArmHardEnv()
        demo = Demo(env, 200, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_parallel("../models/mobile_arm/mobile_arm.urdf", 1.25, rb.MobileArm)

    elif DEMO_SELECTION == 5:
        # mobile arm with observation point demo
        env = environments.MobileArmObservationPointEnv()
        demo = Demo(env, 200, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_parallel("../models/mobile_arm/mobile_arm.urdf", 1.25, rb.MobileArm)

    elif DEMO_SELECTION == 6:
        # simple search and rescue demo
        env = environments.SearchAndRescueSimpleEnv()
        demo = Demo(env, 200, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_parallel("../models/mobile_arm/mobile_arm.urdf", 1, rb.MobileArm)

    elif DEMO_SELECTION == 7:
        # search and rescue demo
        env = environments.SearchAndRescueEnv()
        demo = Demo(env, 300, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_parallel("../models/mobile_arm/mobile_arm.urdf", 1, rb.MobileArm)

    elif DEMO_SELECTION == 8:
        # simple parking demo
        env = environments.ParkingEnv()
        demo = Demo(env, 30, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_consecutive()

    elif DEMO_SELECTION == 9:
        # corner parking demo
        env = environments.ParkingCornerEnv()
        demo = Demo(env, 300, -1, 1000)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.demo_consecutive()

    input("Press Enter to continue...")
