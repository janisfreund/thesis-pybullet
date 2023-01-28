import os.path as osp
import pybullet as p
import sys
import pybullet_data

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from my_planar_robot import MyPlanarRobot
from my_planar_robot import MyMobileArm
from my_planar_robot import MySmallMobileArm
from my_planar_robot import MyCar


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
        robot = MyMobileArm(robot_id)

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

        self.add_door([10, 10, 10], [0.001, 0.001, 0.001], [1, 1, 1, 1])
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles, self.poobjects, self.poobjects_properties,
                                                19, [[0], [0], [1]], self.goal_states, "real")
        world = self.pb_ompl_interface.si.getWorld()
        world.setState(0)

        state = [0 for _ in range(robot.num_dim)]
        robot.set_state(state)

        print("-----------------------------")
        print("-----------------------------")
        print(self.pb_ompl_interface.is_state_valid(state, world))
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



if __name__ == '__main__':
    env = BoxDemo()
    input("Press Enter to continue...")