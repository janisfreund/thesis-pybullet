import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import numpy as np

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from pb_ompl import PbOMPLRobot
from robots import MobileArm


class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        # p.loadURDF("plane.urdf", useFixedBase=True)
        # print("Warehouse imported.")
        office = p.loadURDF("../models/office/office.urdf", useFixedBase=True)

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

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0), globalScaling=1.25)
        print("Robot imported")
        # robot = PbOMPLRobot(robot_id)
        robot = MobileArm(robot_id)
        self.robot = robot

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, 10, [[1], [0], [0]])

        # add obstacles
        self.add_obstacles()


    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add targets
        # self.add_door([1., 1.9, 1.1], [0.05, 0.05, 0.05], [1.,0., 0., 1.])
        # self.add_door([-1., 1.9, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        # self.add_door([-3, -4, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        # self.add_door([3, -4, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])

        self.add_door_mesh("../models/dog/dog.urdf", [-4.32, -4.54, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [2.32, -4.74, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [2.05, 4.42, 0], [0, 0, 0, 1], [0., 0., 1., 1.])
        self.add_door_mesh("../models/dog/dog.urdf", [-4.47, -1.58, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

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

    def add_door_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)

        self.poobjects.append(obj)

    def demo(self):
        joint_ids = []
        state = []
        for i in range (self.robot.num_dim):
            if i > 1:
                joint_ids.append(p.addUserDebugParameter('Joint ' + str(i), -math.pi, math.pi, 0.))
            else:
                joint_ids.append(p.addUserDebugParameter('Joint ' + str(i), -5, 5, 0.))
            state.append(0.)
        set_link_id = p.addUserDebugParameter('Camera link ', 0, 25, 10)
        linkid = 10
        while True:
            state_ = []
            for i, id in enumerate(joint_ids):
                state_.append(p.readUserDebugParameter(joint_ids[i]))
            linkid_ = int(p.readUserDebugParameter(set_link_id))
            if (state_ != state or linkid != linkid_):
                self.robot.set_state(state_)
                state = state_

                linkid = linkid_
                camera_orientation = [[0], [0], [1]]

                shape = p.getVisualShapeData(self.robot.id)
                position = p.getLinkState(self.robot.id, linkid)[4]  # 0/4
                r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, linkid)[5])
                r = np.reshape(r_mat, (-1, 3))
                orientation = np.dot(r, camera_orientation).flatten().tolist()
                up = -np.cross(np.array(camera_orientation).flatten(), orientation)
                target = [x + y for x, y in zip(position, orientation)]

                viewMatrix = p.computeViewMatrix(
                    cameraEyePosition=position,
                    cameraTargetPosition=target,
                    cameraUpVector=up)

                projectionMatrix = p.computeProjectionMatrixFOV(
                    fov=45.0,
                    aspect=1.0,
                    nearVal=0.1,
                    farVal=3.1)

                width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                    width=224,
                    height=224,
                    viewMatrix=viewMatrix,
                    projectionMatrix=projectionMatrix)

                p.removeAllUserDebugItems()
                p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)


if __name__ == '__main__':
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")