import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from my_planar_robot import MyPlanarRobot


class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        robot = MyPlanarRobot(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # add obstacles
        self.add_obstacles()

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties, 10, [[1], [0], [0]], self.goal_states, "real")

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

        self.pb_ompl_interface.set_planner("Partial")
        # self.pb_ompl_interface.set_planner("RRT")

        self.pb_ompl_interface.set_state_sampler_name("camera")

        # add camera
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add outer wall
        wall1 = self.add_box([2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        wall2 = self.add_box([-2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        wall3 = self.add_box([0, 2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])
        wall4 = self.add_box([0, -2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])

        # add targets
        self.add_door([-0.3, 0.8, 0.1], [0.1, 1.5, 0.2], [1., 0., 0., 1.])
        # self.add_door([0.4, 1.5, 0.1], [0.1, 0.8, 0.2], [1., 0., 0., 1.])

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

    def demo(self):
        start = [-1.5, 1.5, math.radians(90)]
        goal = [1.5, 1.5, math.radians(0)]

        #visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)
        p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)
        # self.start_robot.set_state(start)
        # self.goal_robot.set_state(goal)
        res, paths, paths_tree = self.pb_ompl_interface.plan(goal)

        # for robot in robots:
        #     robot.set_state(start)

        # print tree
        # if res:
        #     self.pb_ompl_interface.print_tree(paths_tree, 100, False)
        #     return res, paths

        # execute paths in parallel
        if res:
            robots = []
            for _ in paths:
                rid = p.loadURDF("../models/create_description_no_collision/urdf/create_2.urdf", (-1.5, 1.5, 0))
                r = MyPlanarRobot(rid)
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
                # self.pb_ompl_interface.execute_one_after_another(paths, drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                #                                    linkid=19, camera_orientation=[[0], [0], [1]])
            return res, paths


        # execute paths one after another
        # if res:
        #     idx = 0
        #     while True:
        #         path_idx = idx % len(paths)
        #         print("Executing path {}".format(path_idx))
        #         self.pb_ompl_interface.execute(paths[path_idx], camera=True, projectionMatrix=self.projectionMatrix, linkid=10, camera_orientation=[[1], [0], [0]])
        #         idx += 1
        # return res, paths


if __name__ == '__main__':
    # time.sleep(10)
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")