import math
import os.path as osp
import pybullet as p
import sys
import numpy as np

sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
import environments


class Test:
    def __init__(self, env):
        p.setTimeStep(1. / 240.)
        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)
        self.env = env
        self.robot = self.env.robot

        self.pb_ompl_interface = pb_ompl.PbOMPL(self.env.robot, self.env.obstacles, self.env.poobjects,
                                                self.env.poobjects_properties,
                                                self.robot.cam_link_id, self.robot.cam_orientation,
                                                self.env.goal_states, self.env.space_name, self.env.bounds, 0, 0)
        self.pb_ompl_interface.set_obstacles(self.env.obstacles)
        self.world = self.pb_ompl_interface.si.getWorld()
        self.world.setState(self.pb_ompl_interface.si.getWorld().getNumWorldStates() - 1)

    def test(self):
        joint_ids = []
        state = []
        camera_orientation = self.robot.cam_orientation
        for i in range(self.robot.num_dim):
            if i > 1:
                joint_ids.append(p.addUserDebugParameter('Joint ' + str(i), -math.pi, math.pi, 0.))
            elif i == 0:
                joint_ids.append(
                    p.addUserDebugParameter('Joint ' + str(i), self.env.bounds[0][0] - 5, self.env.bounds[0][1] + 5, 0.))
            elif i == 1:
                joint_ids.append(
                    p.addUserDebugParameter('Joint ' + str(i), self.env.bounds[1][0] - 5, self.env.bounds[1][1] + 5, 0.))
            state.append(0.)
        set_link_id = p.addUserDebugParameter('Camera link ', 0, 30, self.robot.cam_link_id)
        linkid = self.robot.cam_link_id
        b = True
        print("-----------------------------")
        print("Start valid:")
        self.robot.set_state(self.env.start)
        print(self.pb_ompl_interface.is_state_valid(self.env.start, self.world))
        for n, g in enumerate(self.env.goal_states):
            print("-----------------------------")
            print("Goal " + str(n) + " valid:")
            self.robot.set_state(g)
            print(self.pb_ompl_interface.is_state_valid(g, self.world))
        print("-----------------------------")
        while True:
            state_ = []
            for i, id in enumerate(joint_ids):
                state_.append(p.readUserDebugParameter(joint_ids[i]))
            linkid_ = int(p.readUserDebugParameter(set_link_id))
            if (state_ != state or linkid != linkid_ or b):
                b = False
                self.robot.set_state(state_)
                state = state_
                linkid = linkid_
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
                p.getCameraImage(
                    width=224,
                    height=224,
                    viewMatrix=viewMatrix,
                    projectionMatrix=self.projectionMatrix)
                p.removeAllUserDebugItems()
                p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)
                p.addUserDebugText("Valid: " + str(self.pb_ompl_interface.is_state_valid(state, self.world)), [0, 0, 3],
                                   [0, 0, 0], 2)
                print("-----------------------------")
                print(self.pb_ompl_interface.is_state_valid(state, self.world))
                print("X: " + str(state[0]) + ", Y:" + str(state[1]))
                print("-----------------------------")


if __name__ == '__main__':
    p.connect(p.GUI)
    env = environments.ParkingCornerEnv()
    test = Test(env)
    test.test()
