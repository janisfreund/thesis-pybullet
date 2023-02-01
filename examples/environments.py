import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import robots


class Environment(object):
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

    def add_obstacle_box(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)
        self.obstacles.append(box_id)
        return box_id

    def add_obstacle_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)
        self.obstacles.append(obj)

    def add_poobject_box(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)
        self.poobjects.append(box_id)
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def add_poobject_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)
        self.poobjects.append(obj)
        self.poobjects_properties.append([obj, pos])

    def add_goal_state(self, state):
        goal = pb_ompl.ou.vectorDouble()
        for f in state:
            goal.append(f)
        self.goal_states.append(goal)


class RoombaEnv(Environment):
    def __init__(self):
        super().__init__()

        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        robot = robots.Roomba(robot_id)
        self.robot = robot

        self.start = [-1.5, 1.5, math.radians(90)]
        self.goal = [1.5, 1.5, math.radians(0)]

        self.add_obstacle_box([2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([-2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([0, 2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([0, -2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])

        self.add_poobject_box([-0.3, 0.8, 0.1], [0.1, 1.5, 0.2], [1., 0., 0., 1.])
        self.add_poobject_box([0.4, 1.5, 0.1], [0.1, 0.8, 0.2], [0., 1., 0., 1.])
