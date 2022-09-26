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
from my_planar_robot import MyPlanarRobot


class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadSDF("../models/warehouse/model.sdf")
        print("Warehouse imported.")

        # load robot
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

        robot_id = p.loadURDF("combined.urdf", (0, 0, 0))
        print("Robot imported")
        robot = MyPlanarRobot(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        time.sleep(10)

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, 10, [[1], [0], [0]])

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
        self.add_door([3, 2, 1.1], [0.05, 0.05, 0.05], [0., 1., 0., 1.])
        self.add_door([-3, -4, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])

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
        return box_id

    def demo(self):
        start = [-1.5, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        goal = [1.5, 1.5, math.radians(-90), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        #visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)
        p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)
        # self.start_robot.set_state(start)
        # self.goal_robot.set_state(goal)
        res, paths = self.pb_ompl_interface.plan(goal)
        if res:
            idx = 0
            while True:
                path_idx = idx % len(paths)
                print("Executing path {}".format(path_idx))
                self.pb_ompl_interface.execute(paths[path_idx], camera=True, projectionMatrix=self.projectionMatrix, linkid=8, camera_orientation=[[0], [0], [1]])
                idx += 1
        return res, paths


if __name__ == '__main__':
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")