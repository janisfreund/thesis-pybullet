import faulthandler
import os.path as osp
import time

import pybullet as p
import math
import sys
import pybullet_data
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import numpy as np

import pkgutil
egl = pkgutil.get_loader('eglRenderer')

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from robots import Car


class BoxDemo():
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

        p.connect(p.GUI)
        p.setTimeStep(1. / 240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())

        # warehouse_ids = p.loadSDF("../models/warehouse_no_ground/model.sdf")
        # p.resetBasePositionAndOrientation(warehouse_ids[0], (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
        # p.loadURDF("plane.urdf", useFixedBase=True)
        p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)

        # colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.3, 0.2])
        # robot_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=[0, 0, 0])

        # robot_id = p.loadURDF("../models/car/car.urdf", globalScaling=2)
        robot_id = p.loadURDF("../models/car/car_jeep.urdf")

        print("Robot imported")
        robot = Car(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # time.sleep(10)

        # add obstacles
        self.add_obstacles()

        # setup pb_ompl
        # self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, 10, [[1], [0], [0]])
        # for mobile arm
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties, 0, [[1], [0], [0]], self.goal_states, "car")
        # self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties,
        #                                         10, [[1], [0], [0]], self.goal_states, "car")

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

        self.pb_ompl_interface.set_planner("Partial")
        # self.pb_ompl_interface.set_planner("RRT")

        self.define_goal_states()

        self.pb_ompl_interface.set_state_sampler_name("camera")

        # add camera
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

    def define_goal_states(self):
        goal1 = pb_ompl.ou.vectorDouble()
        goal1.append(2.5)
        goal1.append(0.8)
        goal1.append(0.5 * math.pi)
        self.goal_states.append(goal1)

        goal2 = pb_ompl.ou.vectorDouble()
        goal2.append(3.5)
        goal2.append(0.8)
        goal2.append(0.5 * math.pi)
        self.goal_states.append(goal2)

        goal3 = pb_ompl.ou.vectorDouble()
        goal3.append(4.5)
        goal3.append(0.8)
        goal3.append(0.5 * math.pi)
        self.goal_states.append(goal3)

        goal4 = pb_ompl.ou.vectorDouble()
        goal4.append(5.5)
        goal4.append(0.8)
        goal4.append(0.5 * math.pi)
        self.goal_states.append(goal4)

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def add_obstacles(self):
        # add parking lots
        self.add_box([2, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_box([3, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_box([2.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)

        self.add_box([4, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_box([3.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)

        self.add_box([5, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_box([4.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)

        self.add_box([6, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_box([5.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)


        # add street limits
        self.add_box([3, 2, 0.25], [5, 0.2, 0.25], [0.5, 0.5, 0.5, 1], True)
        self.add_box([3, -3, 0.25], [5, 0.2, 0.25], [0.5, 0.5, 0.5, 1], True)


        # add parked cars
        # self.add_door([2.5, 0.8, 0.1], [0.3, 0.6, 0.1], [1., 0., 0., 1.])
        # self.add_door([3.5, 0.8, 0.1], [0.3, 0.6, 0.1], [0., 1., 0., 1.])
        # self.add_door([4.5, 0.8, 0.1], [0.3, 0.6, 0.1], [0., 0., 1., 1.])
        # self.add_door([5.5, 0.8, 0.1], [0.3, 0.6, 0.1], [0., 0., 0., 1.])

        self.add_door_mesh("../models/car/car_jeep_no_cam.urdf", [2.5, 0.8, 0], [0, 0, 1, 1], [1., 0., 0., 1.])
        self.add_door_mesh("../models/car/car_jeep_no_cam.urdf", [3.5, 0.8, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_door_mesh("../models/car/car_jeep_no_cam.urdf", [4.5, 0.8, 0], [0, 0, 1, 1], [0., 0., 1., 1.])
        self.add_door_mesh("../models/car/car_jeep_no_cam.urdf", [5.5, 0.8, 0], [0, 0, 1, 1], [0., 0., 0., 1.])


        # add mesh environment
        # wh_info = p.getJointInfo(self.warehouse)
        # print('Warehouse:')
        # print(wh_info)


    def add_box(self, box_pos, half_box_size, color, collision):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        if collision:
            self.obstacles.append(box_id)
        return box_id

    def add_door(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.poobjects.append(box_id)
        # self.poobjects_properties.append([box_pos, half_box_size, color])
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def add_door_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)

        self.poobjects.append(obj)
        self.poobjects_properties.append([obj, pos])

    def demo(self):
        # start = [0, 0, 0, 0, 0, 0, 0, 0, 1.6, 0]
        # goal = [1.5, 1.5, math.radians(-90), 0, 0, 0, 0, 0, math.radians(180), 0]
        start = [0, 0, 0]
        goal = [2.5, 0.8, math.radians(90)]

        #visualize start and goal pose
        p.addUserDebugPoints(pointPositions=[[start[0], start[1], 0]], pointColorsRGB=[[0,1,1]], pointSize=15, lifeTime=0)
        # p.addUserDebugPoints(pointPositions=[[goal[0], goal[1], 0]], pointColorsRGB=[[0, 0, 1]], pointSize=15, lifeTime=0)

        self.robot.set_state(start)

        # time.sleep(10)

        # self.start_robot.set_state(start)
        # self.goal_robot.set_state(goal)
        res, paths, paths_tree = self.pb_ompl_interface.plan(goal)
        if res:
            # robots = []
            # for _ in paths:
            #     rid = p.loadURDF("../models/car/car.urdf", (0, 0, 0), globalScaling=2)
            #     r = MyCar(rid)
            #     robots.append(r)
            for poo in self.poobjects:
                p.setCollisionFilterGroupMask(poo, -1, 0, 0)
            drawPath = True
            stepParam = ""
            while True:
                # stepParam = self.pb_ompl_interface.execute_all(paths, drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                #                                    linkid=0, camera_orientation=[[1], [0], [0]], robots=robots, stepParam=stepParam)
                self.pb_ompl_interface.execute_one_after_another(paths, drawPath, camera=False, projectionMatrix=self.projectionMatrix,
                                                   linkid=19, camera_orientation=[[0], [0], [1]])
                # drawPath = False
            return res, paths



    def test(self):
        joint_ids = []
        state = []
        for i in range (self.robot.num_dim):
            joint_ids.append(p.addUserDebugParameter('Joint ' + str(i), -math.pi, math.pi, 0.))
            state.append(0.)
        set_link_id = p.addUserDebugParameter('Camera link ', 0, 1, 0)
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
                camera_orientation = [[1], [0], [0]]

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
    faulthandler.enable()
    # time.sleep(10)
    env = BoxDemo()
    # env.test()
    env.demo()
    input("Press Enter to continue...")