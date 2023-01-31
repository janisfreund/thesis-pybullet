import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from my_planar_robot import MyMobileArm


class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

        p.connect(p.GUI)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        office = p.loadURDF("../models/office/office.urdf", useFixedBase=True)
        self.obstacles.append(office)

        # load robot
        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0))
        robot = MyMobileArm(robot_id)
        self.robot = robot

        # add obstacles
        self.add_obstacles()

        self.define_goal_states()

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties, 19, [[0], [0], [1]], self.goal_states, "real")

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

        self.pb_ompl_interface.set_planner("Partial")

        self.pb_ompl_interface.set_state_sampler_name("camera")

        # add camera
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

    def define_goal_states(self):
        goal1 = pb_ompl.ou.vectorDouble()
        goal1.append(-4.579)
        goal1.append(-3.684)
        goal1.append(-1.356)
        goal1.append(0)
        goal1.append(0.893)
        goal1.append(0)
        goal1.append(-1.025)
        goal1.append(0)
        goal1.append(1.918)
        goal1.append(0)
        self.goal_states.append(goal1)

        goal2 = pb_ompl.ou.vectorDouble()
        goal2.append(1.421)
        goal2.append(-4.684)
        goal2.append(0)
        goal2.append(0)
        goal2.append(0.893)
        goal2.append(0)
        goal2.append(-1.025)
        goal2.append(0)
        goal2.append(1.918)
        goal2.append(0)
        self.goal_states.append(goal2)

        goal3 = pb_ompl.ou.vectorDouble()
        goal3.append(2)
        goal3.append(3.474)
        goal3.append(1.554)
        goal3.append(0)
        goal3.append(0.893)
        goal3.append(0)
        goal3.append(-1.025)
        goal3.append(0)
        goal3.append(1.918)
        goal3.append(0)
        self.goal_states.append(goal3)

        goal4 = pb_ompl.ou.vectorDouble()
        goal4.append(-3.421)
        goal4.append(-1.579)
        goal4.append(3.142)
        goal4.append(0)
        goal4.append(0.893)
        goal4.append(0)
        goal4.append(-1.025)
        goal4.append(0)
        goal4.append(1.918)
        goal4.append(0)
        self.goal_states.append(goal4)

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add targets
        self.add_door_mesh("../models/dog/dog.urdf", [-4.32, -4.54, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [2.32, -4.74, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [2.05, 4.42, 0], [0, 0, 0, 1], [0., 0., 1., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [-4.47, -1.58, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

    def add_box(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def add_door(self,  box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.poobjects.append(box_id)
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def add_door_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)

        self.poobjects.append(obj)
        self.poobjects_properties.append([obj, pos])

    def demo(self):
        start = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        goal = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        #visualize start and goal pose
        # p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)
        res, paths, paths_tree = self.pb_ompl_interface.plan(goal)

        # execute paths in parallel
        if res:
            robots = []
            for _ in paths:
                rid = p.loadURDF("../models/mobile_arm/mobile_arm.urdf")
                r = MyMobileArm(rid)
                robots.append(r)
            drawPath = True
            stepParam = ""
            raw_path_param = ""
            sol_line_ids = []
            line_id = []
            while True:
                stepParam, raw_path_param, sol_line_ids, line_id = self.pb_ompl_interface.execute_all(paths, drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                                                   linkid=19, camera_orientation=[[0], [0], [1]], robots=robots, stepParam=stepParam, raw_path_param=raw_path_param,
                                                   sol_line_ids=sol_line_ids, line_id=line_id)
            return res, paths

if __name__ == '__main__':
    # time.sleep(10)
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")