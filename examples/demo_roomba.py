import os.path as osp
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

        # TODO add start and goal for visualization
        # start_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", (0, 0, 0), useFixedBase=1)
        # self.start_robot = pb_ompl.PbOMPLRobot(start_id)
        # goal_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", (0, 0, 0), useFixedBase=1)
        # self.goal_robot = pb_ompl.PbOMPLRobot(goal_id)

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, 10, [[1], [0], [0]])

        self.pb_ompl_interface.set_planner("Partial")
        # self.pb_ompl_interface.set_planner("RRT")

        # add obstacles
        self.add_obstacles()

        # add camera
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=3.1)

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add outer wall
        wall1 = self.add_box([5, 0, 1], [0.1, 5, 1])
        wall2 = self.add_box([-5, 0, 1], [0.1, 5, 1])
        wall3 = self.add_box([0, 5, 1], [5, 0.1, 1])
        wall4 = self.add_box([0, -5, 1], [5, 0.1, 1])

        # add obstacles
        self.add_box([2, 2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([-2, 2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([2, -2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([-2, -2, 0.25], [0.5, 0.5, 0.5])

        # add targets
        self.add_target([1.5, 0, 0.25], 0.1)

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)


    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def add_target(self, box_pos, radius):
        colBoxId = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[0., 1., 0., 1.])
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def demo(self):
        start = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        goal = [-1, 3, math.radians(-90), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        #visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)
        p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)
        # self.start_robot.set_state(start)
        # self.goal_robot.set_state(goal)
        res, path = self.pb_ompl_interface.plan(goal)
        if res:
            while True:
                self.pb_ompl_interface.execute(path, camera=True, projectionMatrix=self.projectionMatrix, linkid=10, camera_orientation=[[1], [0], [0]])
        return res, path


if __name__ == '__main__':
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")