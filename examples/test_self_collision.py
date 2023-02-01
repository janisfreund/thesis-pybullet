import os.path as osp
import time

import numpy as np
import pybullet as p
import sys
import pybullet_data

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from robots import Roomba
from robots import MobileArm
from robots import SmallMobileArm
from robots import Car


class BoxDemo():
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # robot_id = p.loadURDF("combined.urdf", (0, 0, 0), globalScaling=1.25)
        # robot = MyMobileArm(robot_id)

        # robot_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", globalScaling=1.25)
        # # robot = MySmallMobileArm(robot_id)
        # robot = pb_ompl.PbOMPLRobot(robot_id)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0), globalScaling=1.25)
        robot = MobileArm(robot_id)

        # robot_id = p.loadURDF("../models/mobile_arm/mobile_arm_test.urdf", globalScaling=1.25)
        # robot = pb_ompl.PbOMPLRobot(robot_id)

        # robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        # robot = MyPlanarRobot(robot_id)

        # robot_id = p.loadURDF("../models/car/car.urdf", globalScaling=2)
        # robot = MyCar(robot_id)

        print("Robot imported")

        self.robot = robot
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

        # office = p.loadSDF("../models/office/office_no_doors.sdf")

        office = p.loadURDF("../models/office/office.urdf", (0, 0, 0), useFixedBase=True)
        self.obstacles.append(office)

        self.add_mesh("../models/dog/dog.urdf", [-4.32, -4.54, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_mesh("../models/dog/dog.urdf", [2.32, -4.74, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_mesh("../models/dog/dog.urdf", [2.05, 4.42, 0], [0, 0, 0, 1], [0., 0., 1., 1.])
        self.add_mesh("../models/dog/dog.urdf", [-4.47, -1.58, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

        # warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        # self.obstacles.append(warehouse)

        self.add_door([10, 10, 10], [0.001, 0.001, 0.001], [1, 1, 1, 1])
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties,
                                                19, [[0], [0], [1]], self.goal_states, "real")
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        world = self.pb_ompl_interface.si.getWorld()
        world.setState(0)

        state = [0 for _ in range(robot.num_dim)]

        goal1 = []
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

        goal2 = []
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

        goal3 = []
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

        goal4 = []
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

        state = goal4

        robot.set_state(state)
        print("-----------------------------")
        print("-----------------------------")
        print(self.pb_ompl_interface.is_state_valid(state, world))
        print("-----------------------------")
        print("-----------------------------")

        time.sleep(10)

        old_x = 0
        old_y = 0
        x = p.addUserDebugParameter("x", -5, 5, 0)
        y = p.addUserDebugParameter("y", -5, 5, 0)

        # for x in np.arange(-5, 5, 0.1):
        #     for y in np.arange(-5, 5, 0.1):
        #         robot.set_state([x, y, 0])
        #         print(self.pb_ompl_interface.is_state_valid(state, world))

        while True:
            if old_x != p.readUserDebugParameter(x) or old_y != p.readUserDebugParameter(y):
                old_x = p.readUserDebugParameter(x)
                old_y = p.readUserDebugParameter(y)
                state[0] = old_x
                state[1] = old_y
                robot.set_state(state)
                print("-----------------------------")
                print("-----------------------------")
                print(self.pb_ompl_interface.is_state_valid(state, world))
                print("X: " + str(old_x) + ", Y:" + str(old_y))
                print("-----------------------------")
                print("-----------------------------")

    def add_door(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)

        self.poobjects.append(box_id)
        # self.poobjects_properties.append([box_pos, half_box_size, color])
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def add_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)

        self.obstacles.append(obj)



if __name__ == '__main__':
    env = BoxDemo()
    input("Press Enter to continue...")