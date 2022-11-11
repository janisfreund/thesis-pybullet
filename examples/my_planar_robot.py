import pybullet as p
from scipy.spatial.transform import Rotation as R
import math

from pb_ompl import PbOMPLRobot

class MyPlanarRobot(PbOMPLRobot):
    def __init__(self, id) -> None:
        # self.id = id
        # self.num_dim = 13
        # self.joint_idx=[0,1,2,3,4,5,6,7,8,9]
        # self.reset()
        #
        # self.joint_bounds = []
        # self.joint_bounds.append([-5, 5]) # x
        # self.joint_bounds.append([-5, 5]) # y
        # self.joint_bounds.append([math.radians(-180), math.radians(180)]) # theta
        # self.joint_bounds.append([0,1]) # joint_0
        # self.joint_bounds.append([0,1]) # joint_1
        # self.joint_bounds.append([0,1]) # joint_2
        # self.joint_bounds.append([0,1]) # joint_3
        # self.joint_bounds.append([0,1])  # joint_4
        # self.joint_bounds.append([0,1])  # joint_5
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])  # joint_6
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])  # joint_7
        # self.joint_bounds.append([0,1])  # joint_8
        # self.joint_bounds.append([0,1])  # joint_9

        self.id = id
        self.num_dim = 3
        self.joint_idx = [0, 1, 2]
        self.reset()

        self.joint_bounds = []
        self.joint_bounds.append([-5, 5])  # x
        self.joint_bounds.append([-5, 5])  # y
        self.joint_bounds.append([math.radians(-180), math.radians(180)])  # theta

    def get_joint_bounds(self):
        return self.joint_bounds

    def get_cur_state(self):
        return self.state

    def set_state(self, state):
        pos = [state[0], state[1], 0]
        r = R.from_euler('z', state[2])
        quat = r.as_quat()
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        self._set_joint_positions(self.joint_idx, state[3:])

        self.state = state

    def reset(self):
        p.resetBasePositionAndOrientation(self.id, [0,0,0], [0,0,0,1])
        self._set_joint_positions(self.joint_idx, [0,0,0,0])
        self.state = [0] * self.num_dim

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)


class MyMobileArm(PbOMPLRobot):
    def __init__(self, id) -> None:
        self.id = id

        # only 0,1,2,4,8 are important
        # self.joint_idx = [12, 13, 14, 15, 16, 17, 18]
        # self.num_dim = 10
        self.joint_idx = [13, 17]
        self.num_dim = 5

        self.reset()

        self.joint_bounds = []
        self.joint_bounds.append([-5, 5])  # x
        self.joint_bounds.append([-5, 5])  # y
        self.joint_bounds.append([math.radians(-180), math.radians(180)])  # theta
        self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # self.joint_bounds.append([math.radians(-180), math.radians(180)])

    def get_joint_bounds(self):
        return self.joint_bounds

    def get_cur_state(self):
        return self.state

    def set_state(self, state):
        pos = [state[0], state[1], 0]
        r = R.from_euler('z', state[2])
        quat = r.as_quat()
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        self._set_joint_positions(self.joint_idx, state[3:])

        self.state = state

    def reset(self):
        p.resetBasePositionAndOrientation(self.id, [0,0,0], [0,0,0,1])
        self._set_joint_positions(self.joint_idx, [0,0,0,0])
        self.state = [0] * self.num_dim

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)