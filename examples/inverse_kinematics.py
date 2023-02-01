import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from robots import Roomba
from robots import MobileArm

GOAL = 1
# TARGET = []
# TARGET = [1.7891253349769771, -6.208575926370699, 1.1]
TARGET = [-2.260535103873994, 3.435942762254409, 1.1]

class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

        self.goalPos = [[3, 1.9, 1.1], [-3, 1.9, 1.1], [-3, -4, 1.1], [3, -4, 1.1]]

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # warehouse_ids = p.loadSDF("../models/warehouse_no_ground/model.sdf")
        # p.resetBasePositionAndOrientation(warehouse_ids[0], (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
        self.warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        p.loadURDF("plane.urdf", useFixedBase=True)
        print("Warehouse imported.")

        robot_id = p.loadURDF("combined.urdf", (0, 0, 0), globalScaling=1.25)
        print("Robot imported")
        robot = MobileArm(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # setup pb_ompl
        # self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, 10, [[1], [0], [0]])
        # for mobile arm
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties, 19, [[0], [0], [1]], self.goal_states)

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
        # add targets
        if len(TARGET) == 0:
            self.add_door(self.goalPos[GOAL], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        else:
            self.add_door(TARGET, [0.05, 0.05, 0.05], [1., 0., 0., 1.])

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)


    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def add_door(self,  box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.poobjects.append(box_id)
        # self.poobjects_properties.append([box_pos, half_box_size, color])
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def demo(self):
        start = [0, 0, 0, 0, 0]
        goal = [1.5, 1.5, math.radians(-90), 0, math.radians(180)]

        #visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)
        p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)

        if len(TARGET) == 0:
            self.pb_ompl_interface.sample_good_camera_position(self.goalPos[GOAL], [0, 0, 0], 1, 19)
        else:
            self.pb_ompl_interface.sample_good_camera_position(TARGET, [0, 0, 0], 1, 19)


if __name__ == '__main__':
    # time.sleep(10)
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")