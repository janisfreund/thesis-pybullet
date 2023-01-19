try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'thesis-ompl/ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    # print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
import operator
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class CameraStateSampler(ob.RealVectorStateSampler):
    def __init__(self, si, base_offset, camera_link_id, robot, poobjects_pos):
        super().__init__(si.getStateSpace())
        self.si = si
        self.name_ = "Camera State Sampler"
        self.rng_ = ou.RNG()
        self.rng_.setSeed(0)
        self.rng_.setLocalSeed(0)
        self.base_offset = base_offset
        self.camera_link_id = camera_link_id
        self.robot = robot
        self.robot_id = robot.id
        self.poobjects_pos = poobjects_pos


    def sampleGoodCameraPosition(self, state):
        # rstate = self.state_to_list(state)
        # sample uniformly in 50% of the cases

        # call uniform sampling directly in ompl planner
        # random_num = self.rng_.uniformReal(0, 1)
        # if random_num < 0.5:
        #     super().sampleUniform(state)
        # else:
        bounds = self.robot.get_joint_bounds()
        base_pos = [self.rng_.uniformReal(bounds[0][0], bounds[0][1]), self.rng_.uniformReal(bounds[1][0], bounds[1][1])]
        world = self.si.getWorld()
        obj_pos = self.poobjects_pos[world.getStateIdx(world.getStateInt())]
        # position is directly above base + offset in z direction
        # offset = p.getLinkState(self.robot_id, self.camera_link_id)[4][2]
        position = [0, 0, self.base_offset]
        final_position = [base_pos[0], base_pos[1], self.base_offset]
        target = obj_pos.copy()
        target[0] = obj_pos[0] - base_pos[0]
        target[1] = obj_pos[1] - base_pos[1]
        direction = list(map(operator.sub, target, position))
        direction = direction / np.linalg.norm(direction)
        up = [0, 0, 1]

        xaxis = np.cross(up, direction)
        xaxis = xaxis / np.linalg.norm(xaxis)

        yaxis = np.cross(direction, xaxis)
        yaxis = yaxis / np.linalg.norm(yaxis)

        mat = np.array([xaxis, yaxis, direction]).transpose()
        quaternion = R.from_matrix(mat).as_quat()

        p.resetBasePositionAndOrientation(self.robot.id, [0, 0, 0], [0, 0, 0, 1])
        inv = p.calculateInverseKinematics(self.robot.id, self.camera_link_id, position, targetOrientation=quaternion,
                                           maxNumIterations=100)

        state[0] = base_pos[0]
        state[1] = base_pos[1]
        state[2] = 0
        for i, joint_pos in enumerate(inv):
            state[3 + i] = joint_pos

        print("Sampled camera pos.")

        # debugging
        # debugState = []
        # debugState.append(base_pos[0])
        # debugState.append(base_pos[1])
        # debugState.append(0)
        # for i, joint_pos in enumerate(inv):
        #     debugState.append(joint_pos)
        #
        # self.robot.set_state(debugState)
        #
        # position_cam = p.getLinkState(self.robot.id, self.camera_link_id)[4]
        # r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, self.camera_link_id)[5])
        # r = np.reshape(r_mat, (-1, 3))
        # orientation = np.dot(r, [[0], [0], [1]]).flatten().tolist()
        # debug_target = [x + y for x, y in zip(position_cam, orientation)]
        #
        # debug_orientation = np.dot(mat, [[0], [0], [1]]).flatten().tolist()
        # debug_point = [x + y for x, y in zip(final_position, debug_orientation)]
        #
        # p.removeAllUserDebugItems()
        # p.addUserDebugLine(final_position, obj_pos, lineColorRGB=[1, 0, 0], lineWidth=5)
        # p.addUserDebugLine(final_position, debug_point, lineColorRGB=[0, 0, 1], lineWidth=5)
        # p.addUserDebugLine(position_cam, debug_target, lineColorRGB=[0, 1, 0], lineWidth=5)
        #
        # time.sleep(2.5)

        return True


    def sampleGoodCameraPositionNear(self, state, x, y):
        base_pos = [x, y]
        world = self.si.getWorld()
        obj_pos = self.poobjects_pos[world.getStateIdx(world.getStateInt())]
        position = [0, 0, self.base_offset]
        final_position = [base_pos[0], base_pos[1], self.base_offset]
        target = obj_pos.copy()
        target[0] = obj_pos[0] - base_pos[0]
        target[1] = obj_pos[1] - base_pos[1]
        direction = list(map(operator.sub, target, position))
        direction = direction / np.linalg.norm(direction)
        up = [0, 0, 1]

        xaxis = np.cross(up, direction)
        xaxis = xaxis / np.linalg.norm(xaxis)

        yaxis = np.cross(direction, xaxis)
        yaxis = yaxis / np.linalg.norm(yaxis)

        mat = np.array([xaxis, yaxis, direction]).transpose()
        quaternion = R.from_matrix(mat).as_quat()

        p.resetBasePositionAndOrientation(self.robot.id, [0, 0, 0], [0, 0, 0, 1])
        inv = p.calculateInverseKinematics(self.robot.id, self.camera_link_id, position, targetOrientation=quaternion,
                                           maxNumIterations=100)

        state[0] = base_pos[0]
        state[1] = base_pos[1]
        state[2] = 0
        for i, joint_pos in enumerate(inv):
            state[3 + i] = joint_pos

        print("Sampled camera pos.")

        # debugging
        # debugState = []
        # debugState.append(base_pos[0])
        # debugState.append(base_pos[1])
        # debugState.append(0)
        # for i, joint_pos in enumerate(inv):
        #     debugState.append(joint_pos)
        #
        # self.robot.set_state(debugState)
        #
        # position_cam = p.getLinkState(self.robot.id, self.camera_link_id)[4]
        # r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, self.camera_link_id)[5])
        # r = np.reshape(r_mat, (-1, 3))
        # orientation = np.dot(r, [[0], [0], [1]]).flatten().tolist()
        # debug_target = [x + y for x, y in zip(position_cam, orientation)]
        #
        # debug_orientation = np.dot(mat, [[0], [0], [1]]).flatten().tolist()
        # debug_point = [x + y for x, y in zip(final_position, debug_orientation)]
        #
        # p.removeAllUserDebugItems()
        # p.addUserDebugLine(final_position, obj_pos, lineColorRGB=[1, 0, 0], lineWidth=5)
        # p.addUserDebugLine(final_position, debug_point, lineColorRGB=[0, 0, 1], lineWidth=5)
        # p.addUserDebugLine(position_cam, debug_target, lineColorRGB=[0, 1, 0], lineWidth=5)
        #
        # time.sleep(2.5)

        # time.sleep(10)

        return True


    def state_to_list(self, state):
        rstate = []
        for s in state:
            rstate.append(s)
        return rstate