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

from ompl import tools as t

import faulthandler


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
        # warehouse_ids = p.loadSDF("../models/warehouse_no_ground/model.sdf")
        # p.resetBasePositionAndOrientation(warehouse_ids[0], (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
        self.warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        p.loadURDF("plane.urdf", useFixedBase=True)
        print("Warehouse imported.")

        # load robot
        if False:
            # modified from https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_utils/examples/combineUrdf.py
            p0 = bc.BulletClient(connection_mode=p.DIRECT)
            p0.setAdditionalSearchPath(pybullet_data.getDataPath())

            p1 = bc.BulletClient(connection_mode=p.DIRECT)
            p1.setAdditionalSearchPath(pybullet_data.getDataPath())

            # can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

            roomba = p1.loadURDF("../models/create_description/urdf/create_2.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)
            franka = p0.loadURDF("../models/franka_description/robots/panda_arm.urdf")

            ed0 = ed.UrdfEditor()
            ed0.initializeFromBulletBody(roomba, p1._client)
            ed1 = ed.UrdfEditor()
            ed1.initializeFromBulletBody(franka, p0._client)

            parentLinkIndex = 0

            jointPivotXYZInParent = [0, 0, 0]
            jointPivotRPYInParent = [0, 0, 0]

            jointPivotXYZInChild = [0, 0, 0]
            jointPivotRPYInChild = [0, 0, 0]

            newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                                    jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
            newjoint.joint_type = p0.JOINT_FIXED

            ed0.saveUrdf("combined.urdf")

        robot_id = p.loadURDF("combined.urdf", (0, 0, 0), globalScaling=1.25)
        print("Robot imported")
        robot = MobileArm(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # time.sleep(10)

        # add obstacles
        self.add_obstacles()

        self.define_goal_states()

        # setup pb_ompl
        # self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, 10, [[1], [0], [0]])
        # for mobile arm
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties, 19, [[0], [0], [1]], self.goal_states, "real")

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

        # self.pb_ompl_interface.set_planner("Partial")
        # self.pb_ompl_interface.set_planner("RRT")

        self.pb_ompl_interface.set_state_sampler_name("camera")

        # add camera
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

        start = [0, 0, 0, 0, 0, 0, 0, 0, 1.6, 0]
        goal = [1.5, 1.5, math.radians(-90), 0, 0, 0, 0, 0, math.radians(180), 0]

        # visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0, 1, 1]], pointSize=15,
                             lifeTime=0)
        p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15,
                             lifeTime=0)

        self.robot.set_state(start)

        self.pb_ompl_interface.set_start_goal(start, goal)

    def define_goal_states(self):
        goal1 = pb_ompl.ou.vectorDouble()
        goal1.append(2.844)
        goal1.append(1.124)
        goal1.append(1.455)
        goal1.append(0)
        goal1.append(0.595)
        goal1.append(0)
        goal1.append(0)
        goal1.append(0)
        goal1.append(1.422)
        goal1.append(0)
        self.goal_states.append(goal1)

        goal2 = pb_ompl.ou.vectorDouble()
        goal2.append(-3.042)
        goal2.append(1.124)
        goal2.append(1.455)
        goal2.append(0)
        goal2.append(0.595)
        goal2.append(0)
        goal2.append(0)
        goal2.append(0)
        goal2.append(1.442)
        goal2.append(0)
        self.goal_states.append(goal2)

        goal3 = pb_ompl.ou.vectorDouble()
        goal3.append(-3.042)
        goal3.append(-3.142)
        goal3.append(-1.587)
        goal3.append(0)
        goal3.append(0.595)
        goal3.append(0)
        goal3.append(0)
        goal3.append(0)
        goal3.append(1.422)
        goal3.append(0)
        self.goal_states.append(goal3)

        goal4 = pb_ompl.ou.vectorDouble()
        goal4.append(3.009)
        goal4.append(-3.142)
        goal4.append(-1.587)
        goal4.append(0)
        goal4.append(0.595)
        goal4.append(0)
        goal4.append(0)
        goal4.append(0)
        goal4.append(1.422)
        goal4.append(0)
        self.goal_states.append(goal4)

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add targets
        self.add_door([3, 1.9, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        self.add_door([-3, 1.9, 1.1], [0.05, 0.05, 0.05], [0., 1., 0., 1.])
        self.add_door([-3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 1., 1.])
        self.add_door([3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 0., 1.])

        # add mesh environment
        # wh_info = p.getJointInfo(self.warehouse)
        # print('Warehouse:')
        # print(wh_info)


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

    def benchmark(self):
        # define benchmark
        b = t.Benchmark(self.pb_ompl_interface.ss, "mobile_arm")
        b.addPlanner(pb_ompl.og.Partial(self.pb_ompl_interface.ss.getSpaceInformation()))
        # b.addPlanner(pb_ompl.og.RRT(self.pb_ompl_interface.ss.getSpaceInformation()))

        req = t.Benchmark.Request()
        req.maxTime = 20.0
        req.maxMem = 1000.0
        req.runCount = 10
        req.displayProgress = True

        b.benchmark(req)

        b.saveResultsToFile()


if __name__ == '__main__':
    faulthandler.enable()
    # time.sleep(10)
    env = BoxDemo()
    sys.setrecursionlimit(10000)
    env.benchmark()
    input("Press Enter to continue...")