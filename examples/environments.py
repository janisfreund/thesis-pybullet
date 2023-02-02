import os.path as osp
import pybullet as p
import math
import sys

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import robots


class Environment(object):
    def __init__(self):
        self.obstacles = []
        self.poobjects = []
        self.poobjects_properties = []
        self.goal_states = []

    def add_obstacle_box(self, box_pos, half_box_size, color, collision=True):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)
        if collision:
            self.obstacles.append(box_id)
        return box_id

    def add_obstacle_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)
        self.obstacles.append(obj)

    def add_poobject_box(self, box_pos, half_box_size, color):
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=half_box_size, rgbaColor=color)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, baseCollisionShapeIndex=colBoxId,
                                   basePosition=box_pos)
        self.poobjects.append(box_id)
        self.poobjects_properties.append([visBoxId, colBoxId, box_pos])
        return box_id

    def add_poobject_mesh(self, path, pos, ori, color):
        obj = p.loadURDF(path, pos, ori)
        p.changeVisualShape(obj, -1, rgbaColor=color)
        self.poobjects.append(obj)
        self.poobjects_properties.append([obj, pos])

    def add_goal_state(self, state):
        goal = pb_ompl.ou.vectorDouble()
        for f in state:
            goal.append(f)
        self.goal_states.append(goal)


class RoombaEnv(Environment):
    def __init__(self):
        super().__init__()

        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        robot = robots.Roomba(robot_id)
        self.robot = robot

        self.start = [-1.5, 1.5, math.radians(90)]
        self.goal = [1.5, 1.5, math.radians(0)]

        self.space_name = "real"
        self.bounds = [[-1.7, 1.7], [-1.7, 1.7]]

        self.add_obstacle_box([2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([-2, 0, 0.1], [0.1, 2, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([0, 2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])
        self.add_obstacle_box([0, -2, 0.1], [2, 0.1, 0.2], [0.8, 0.8, 0.8, 1])

        self.add_poobject_box([-0.3, 0.8, 0.1], [0.1, 1.5, 0.2], [1., 0., 0., 1.])
        self.add_poobject_box([0.4, 1.5, 0.1], [0.1, 0.8, 0.2], [0., 1., 0., 1.])


class RoombaDoorEnv(Environment):
    def __init__(self):
        super().__init__()

        floor_plan = p.loadURDF("../models/floor_plan/floor_plan.urdf", useFixedBase=True)
        self.obstacles.append(floor_plan)

        robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        robot = robots.Roomba(robot_id)
        self.robot = robot

        self.start = [3.32, 3.32, 0]
        self.goal = [1.8, 0, -3.142]

        self.space_name = "real"
        self.bounds = [[-4, 4], [-4, 4]]

        self.add_poobject_mesh("../models/house/door.urdf", [2.4, -3, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_poobject_mesh("../models/house/door.urdf", [-2, -0.6, 0], [0, 0, 0, 1], [0., 1., 0., 1.])
        self.add_poobject_mesh("../models/house/door.urdf", [0.2, 0.4, 0], [0, 0, 1, 1], [0., 0., 1., 1.])

        # self.add_poobject_mesh("../models/house/door.urdf", [2.4, 3, 0], [0, 0, 0, 1], [0., 0., 0., 1.])
        # self.add_poobject_mesh("../models/house/door.urdf", [-2, -2.6, 0], [0, 0, 0, 1], [0., 0., 0., 1.])
        # self.add_poobject_mesh("../models/house/door.urdf", [-3, -1.6, 0], [0, 0, 1, 1], [0., 0., 0., 1.])


class RoombaHouseEnv(Environment):
    def __init__(self):
        super().__init__()

        office = p.loadURDF("../models/house/house.urdf", useFixedBase=True)
        self.obstacles.append(office)

        robot_id = p.loadURDF("../models/create_description/urdf/create_2.urdf", (0, 0, 0))
        robot = robots.Roomba(robot_id)
        self.robot = robot

        self.start = [-5.368, -7.684, 0]
        self.goal = [-5.368, 3.263, 0]

        self.space_name = "real"
        self.bounds = [[-7, 7], [-9, 9]]

        self.add_poobject_mesh("../models/dog/dog.urdf", [-4.32, -4.54, 0], [0, 0, 0, 1], [1., 0., 0., 1.])


class MobileArmEnv(Environment):
    def __init__(self):
        super().__init__()

        warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(warehouse)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0), globalScaling=1.25)
        robot = robots.MobileArm(robot_id)
        self.robot = robot

        self.start = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.goal = [1.5, 1.5, math.radians(-90), 0, math.radians(180), 0, 0, 0, 0, 0]

        self.space_name = "real"
        self.bounds = [[-5.5, 3.5], [-5.5, 3.5]]

        self.add_poobject_box([3, 1.9, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        self.add_poobject_box([-3, 1.9, 1.1], [0.05, 0.05, 0.05], [0., 1., 0., 1.])
        self.add_poobject_box([-3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 1., 1.])
        self.add_poobject_box([3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 0., 1.])

        self.add_goal_state([2.844, 1.124, 1.455, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([-3.042, 1.124, 1.455, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([-3.042, -3.142, -1.587, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([3.009, -3.142, -1.587, 0, 0.595, 0, 0, 0, 1.422, 0])


class MobileArmHardEnv(Environment):
    def __init__(self):
        super().__init__()

        warehouse = p.loadURDF("../models/warehouse_no_ground/model.urdf", useFixedBase=True)
        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(warehouse)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0), globalScaling=1.25)
        robot = robots.MobileArm(robot_id)
        self.robot = robot

        self.start = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.goal = [1.5, 1.5, math.radians(-90), 0, math.radians(180), 0, 0, 0, 0, 0]

        self.space_name = "real"
        self.bounds = [[-5.5, 3.5], [-5.5, 3.5]]

        self.add_obstacle_box([-1, 1, 1], [0.5, 0.1, 1], [1., 1., 1., 1.])
        self.add_obstacle_box([-1.5, 0.5, 1], [0.1, 0.5, 1], [1., 1., 1., 1.])
        self.add_obstacle_box([1, 1, 1], [0.5, 0.1, 1], [1., 1., 1., 1.])
        self.add_obstacle_box([1.5, 0.5, 1], [0.1, 0.5, 1], [1., 1., 1., 1.])
        self.add_obstacle_box([0, -1.5, 1], [1.5, 0.1, 1], [1., 1., 1., 1.])

        self.add_poobject_box([3, 1.9, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        self.add_poobject_box([-3, 1.9, 1.1], [0.05, 0.05, 0.05], [0., 1., 0., 1.])
        self.add_poobject_box([-3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 1., 1.])
        self.add_poobject_box([3, -4, 1.1], [0.05, 0.05, 0.05], [0., 0., 0., 1.])

        self.add_goal_state([2.844, 1.124, 1.455, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([-3.042, 1.124, 1.455, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([-3.042, -3.142, -1.587, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([3.009, -3.142, -1.587, 0, 0.595, 0, 0, 0, 1.422, 0])


class MobileArmObservationPointEnv(Environment):
    def __init__(self):
        super().__init__()

        warehouse = p.loadURDF("../models/warehouse_no_ground/warehouse_observation_point.urdf", useFixedBase=True)
        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(warehouse)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0), globalScaling=1.25)
        robot = robots.MobileArm(robot_id)
        self.robot = robot

        self.start = [0, -5, 0, 0, 0, 0, 0, 0, 1.6, 0]
        self.goal = [1.5, 1.5, math.radians(-90), 0, math.radians(180), 0, 0, 0, 0, 0]

        self.space_name = "real"
        self.bounds = [[-5.5, 3.5], [-5.5, 3.5]]

        self.add_obstacle_box([-2.2, -1.5, 1.5], [1.7, 0.1, 1.5], [1., 1., 1., 1.])
        self.add_obstacle_box([2.2, -1.5, 1.5], [1.7, 0.1, 1.5], [1., 1., 1., 1.])
        self.add_obstacle_box([0, -1.5, 0.5], [0.5, 0.1, 0.5], [1., 1., 1., 1.])
        self.add_obstacle_box([0, -1.5, 2.5], [0.5, 0.1, 0.5], [1., 1., 1., 1.])
        self.add_obstacle_box([-3.5, -1.5, 2.5], [1.5, 0.1, 0.5], [1., 1., 1., 1.])
        self.add_obstacle_box([3.5, -1.5, 2.5], [1.5, 0.1, 0.5], [1., 1., 1., 1.])
        self.add_obstacle_box([0, 4, 0.5], [0.1, 5.5, 0.5], [1., 1., 1., 1.])

        self.add_poobject_box([1, 1.9, 1.1], [0.05, 0.05, 0.05], [1., 0., 0., 1.])
        self.add_poobject_box([-1, 1.9, 1.1], [0.05, 0.05, 0.05], [0., 1., 0., 1.])

        self.add_goal_state([0.9, 1.124, 1.455, 0, 0.595, 0, 0, 0, 1.422, 0])
        self.add_goal_state([-1, 1, 1.587, 0, 0.595, 0, 0, 0, 1.521, 0])


class SearchAndRescueSimpleEnv(Environment):
    def __init__(self):
        super().__init__()

        office = p.loadURDF("../models/office_simple/office.urdf", useFixedBase=True)
        self.obstacles.append(office)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0))
        robot = robots.MobileArm(robot_id)
        self.robot = robot

        self.start = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.goal = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.space_name = "real"
        self.bounds = [[-4, 4], [-4, 4]]

        self.add_poobject_mesh("../models/dog/dog.urdf", [-2.1, 1.8, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [-3.5, 1, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [-3, -3.6, 0], [0, 0, 0, 1], [0., 0., 1., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [1.2, -1.3, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

        self.add_goal_state([-2.874, 1.879, 0, 0, 0.893, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([-2.763, 0.442, 2.348, 0, 0.694, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([-2.211, -3.537, 3.142, 0, 0.893, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([2.211, -1.326, 3.142, 0, 0.694, 0, -1.025, 0, 1.918, 0])


class SearchAndRescueEnv(Environment):
    def __init__(self):
        super().__init__()

        office = p.loadURDF("../models/office/office.urdf", useFixedBase=True)
        self.obstacles.append(office)

        robot_id = p.loadURDF("../models/mobile_arm/mobile_arm.urdf", (0, 0, 0))
        robot = robots.MobileArm(robot_id)
        self.robot = robot

        self.start = [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.goal = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.space_name = "real"
        self.bounds = [[-5.5, 5.5], [-5.5, 5.5]]

        self.add_poobject_mesh("../models/dog/dog.urdf", [-4.32, -4.54, 0], [0, 0, 0, 1], [1., 0., 0., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [2.32, -4.74, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [2.05, 4.42, 0], [0, 0, 0, 1], [0., 0., 1., 1.])
        self.add_poobject_mesh("../models/dog/dog.urdf", [-4.47, -1.58, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

        self.add_goal_state([-4.579, -3.684, -1.356, 0, 0.893, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([1.421, -4.684, 0, 0, 0.893, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([2, 3.474, 1.554, 0, 0.893, 0, -1.025, 0, 1.918, 0])
        self.add_goal_state([-3.421, -1.579, 3.142, 0, 0.893, 0, -1.025, 0, 1.918, 0])


class ParkingEnv(Environment):
    def __init__(self):
        super().__init__()

        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/car/car_jeep.urdf", (0, 0, 0))
        robot = robots.Car(robot_id)
        self.robot = robot

        self.start = [0, 0, 0]
        self.goal = [2.5, 0.8, math.radians(90)]

        self.space_name = "car"
        self.bounds = [[-2, 6], [-2, 6]]

        # add parking lots
        self.add_obstacle_box([2, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([3, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([2.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([4, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([3.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([5, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([4.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([6, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([5.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        # add street limits
        self.add_obstacle_box([3, 2, 0.25], [5, 0.2, 0.25], [0.5, 0.5, 0.5, 1])
        self.add_obstacle_box([3, -3, 0.25], [5, 0.2, 0.25], [0.5, 0.5, 0.5, 1])

        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [2.5, 0.8, 0], [0, 0, 1, 1], [1., 0., 0., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [3.5, 0.8, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [4.5, 0.8, 0], [0, 0, 1, 1], [0., 0., 1., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [5.5, 0.8, 0], [0, 0, 1, 1], [0., 0., 0., 1.])

        self.add_goal_state([2.5, 0.8, 0.5 * math.pi])
        self.add_goal_state([3.5, 0.8, 0.5 * math.pi])
        self.add_goal_state([4.5, 0.8, 0.5 * math.pi])
        self.add_goal_state([5.5, 0.8, 0.5 * math.pi])


class ParkingCornerEnv(Environment):
    def __init__(self):
        super().__init__()

        floor = p.loadURDF("../models/floor/floor.urdf", useFixedBase=True)
        self.obstacles.append(floor)

        robot_id = p.loadURDF("../models/car/car_jeep.urdf", (0, 0, 0))
        robot = robots.Car(robot_id)
        self.robot = robot

        self.start = [0, 0, 0]
        self.goal = [2.5, 0.8, math.radians(90)]

        self.space_name = "car"
        self.bounds = [[0, 11], [-10, 3]]

        # add parking lots
        self.add_obstacle_box([2, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([3, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([2.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([4, 0.8, 0], [0.05, 0.8, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([3.5, 1.55, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)

        self.add_obstacle_box([7.5, -5, 0], [0.05, 1, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([8, -4.05, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([8, -5.95, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([7.5, -7, 0], [0.05, 1, 0.05], [0.5, 0.5, 0.5, 1], False)
        self.add_obstacle_box([8, -7.95, 0], [0.5, 0.05, 0.05], [0.5, 0.5, 0.5, 1], False)

        # add street limits
        self.add_obstacle_box([4, 2, 0.5], [6, 0.2, 0.5], [0.5, 0.5, 0.5, 1])
        self.add_obstacle_box([2.5, -3, 0.5], [4.5, 0.2, 0.5], [0.5, 0.5, 0.5, 1])
        self.add_obstacle_box([9.8, -4, 0.5], [0.2, 6, 0.5], [0.5, 0.5, 0.5, 1])
        self.add_obstacle_box([7.2, -6.4, 0.5], [0.2, 3.6, 0.5], [0.5, 0.5, 0.5, 1])

        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [2.5, 0.8, 0], [0, 0, 1, 1], [1., 0., 0., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [3.5, 0.8, 0], [0, 0, 1, 1], [0., 1., 0., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [8, -5, 0], [0, 0, -1, 1], [0., 0., 1., 1.])
        self.add_poobject_mesh("../models/car/car_jeep_no_cam.urdf", [8, -7, 0], [0, 0, -1, 1], [0., 0., 0., 1.])

        self.add_goal_state([2.5, 0.8, 0.5 * math.pi])
        self.add_goal_state([3.5, 0.8, 0.5 * math.pi])
        self.add_goal_state([8, -5, -0.5 * math.pi])
        self.add_goal_state([8, -7, -0.5 * math.pi])
