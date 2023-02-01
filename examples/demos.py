import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data
from typing import Type

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import robots as rb
import environments


def add_debug_point(pos, radius, color):
    visBoxId = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=0.01, rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, basePosition=[pos[0], pos[1], 0.005])


class Demo:
    def __init__(self, env):
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
                                                self.env.goal_states, "real")

        self.pb_ompl_interface.set_obstacles(self.env.obstacles)
        self.pb_ompl_interface.set_planner("Partial")
        self.pb_ompl_interface.set_state_sampler_name("camera")
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

    def demo_parallel(self, model, RobotClass):
        if self.res:
            robots = []
            for _ in self.paths:
                rid = p.loadURDF(model, (self.env.start[0], self.env.start[1], 0))
                r = RobotClass(rid)
                robots.append(r)
            drawPath = True
            stepParam = ""
            raw_path_param = ""
            sol_line_ids = []
            line_id = []
            while True:
                stepParam, raw_path_param, sol_line_ids, line_id = self.pb_ompl_interface.execute_all(self.paths,
                                                   drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                                                   linkid=self.robot.cam_link_id,
                                                   camera_orientation=self.robot.cam_orientation, robots=robots,
                                                   stepParam=stepParam, raw_path_param=raw_path_param,
                                                   sol_line_ids=sol_line_ids, line_id=line_id)


if __name__ == '__main__':
    # time.sleep(10)
    demo_selection = 0
    p.connect(p.GUI)

    if demo_selection == 0:
        # simple roomba demo
        env = environments.RoombaEnv()
        demo = Demo(env)
        demo.plan()
        demo.draw_start([0, 0, 0, 1])
        demo.draw_goal([0, 0, 0, 1])
        demo.demo_parallel("../models/create_description/urdf/create_2.urdf", rb.Roomba)
    input("Press Enter to continue...")
