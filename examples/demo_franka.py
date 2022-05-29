import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
import time
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl

class BoxDemo():
    def __init__(self):
        self.obstacles = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", (0,0,0), useFixedBase=1)
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot

        # TODO add start and goal for visualization
        # start_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", (0, 0, 0), useFixedBase=1)
        # self.start_robot = pb_ompl.PbOMPLRobot(start_id)
        # goal_id = p.loadURDF("../models/franka_description/robots/panda_arm.urdf", (0, 0, 0), useFixedBase=1)
        # self.goal_robot = pb_ompl.PbOMPLRobot(goal_id)

        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        self.pb_ompl_interface.set_planner("Partial")

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
        # add box
        self.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])

        # surroundings
        self.add_box([2, 2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([-2, 2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([2, -2, 0.25], [0.5, 0.5, 0.5])
        self.add_box([-2, -2, 0.25], [0.5, 0.5, 0.5])

        # store obstacles
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def demo(self):
        start = [0,0,0,-1,0,1.5,0]
        goal = [0,1.5,0,-0.1,0,0.2,0]

        self.robot.set_state(start)
        # self.start_robot.set_state(start)
        # self.goal_robot.set_state(goal)
        res, path = self.pb_ompl_interface.plan(goal)
        if res:
            while True:
                self.pb_ompl_interface.execute(path, camera=True, projectionMatrix=self.projectionMatrix, linkid=8, camera_orientation=[[0], [0], [1]])
        return res, path

if __name__ == '__main__':
    time.sleep(10)
    env = BoxDemo()
    env.demo()
    input("Press Enter to continue...")