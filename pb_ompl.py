try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
import utils
import time
from itertools import product
import copy
import numpy as np
import cv2
import os

INTERPOLATE_NUM = 500
DEFAULT_PLANNING_TIME = 60.0

class PbOMPLRobot():
    '''
    To use with Pb_OMPL. You need to construct a instance of this class and pass to PbOMPL.

    Note:
    This parent class by default assumes that all joints are acutated and should be planned. If this is not your desired
    behaviour, please write your own inheritated class that overrides respective functionalities.
    '''
    def __init__(self, id) -> None:
        # Public attributes
        self.id = id

        # prune fixed joints
        all_joint_num = p.getNumJoints(id)
        all_joint_idx = list(range(all_joint_num))
        joint_idx = [j for j in all_joint_idx if self._is_not_fixed(j)]
        self.num_dim = len(joint_idx)
        self.joint_idx = joint_idx
        print(self.joint_idx)
        self.joint_bounds = []

        self.reset()

    def _is_not_fixed(self, joint_idx):
        joint_info = p.getJointInfo(self.id, joint_idx)
        return joint_info[2] != p.JOINT_FIXED

    def get_joint_bounds(self):
        '''
        Get joint bounds.
        By default, read from pybullet
        '''
        for i, joint_id in enumerate(self.joint_idx):
            joint_info = p.getJointInfo(self.id, joint_id)
            low = joint_info[8] # low bounds
            high = joint_info[9] # high bounds
            if low < high:
                self.joint_bounds.append([low, high])
        print("Joint bounds: {}".format(self.joint_bounds))
        return self.joint_bounds

    def get_cur_state(self):
        return copy.deepcopy(self.state)

    def set_state(self, state):
        '''
        Set robot state.
        To faciliate collision checking
        Args:
            state: list[Float], joint values of robot
        '''
        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def reset(self):
        '''
        Reset robot state
        Args:
            state: list[Float], joint values of robot
        '''
        state = [0] * self.num_dim
        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)

# TODO change to new state space
class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        '''
        This will be called by the internal OMPL planner
        '''
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        '''
        Optional, Set custom state sampler.
        '''
        self.state_sampler = state_sampler

class PbOMPL():
    def __init__(self, robot, obstacles = [], poobjects = [], camera_link = 10, camera_orientation = [[1], [0], [0]]) -> None:
        '''
        Args
            robot: A PbOMPLRobot instance.
            obstacles: list of obstacle ids. Optional.
        '''
        self.robot = robot
        self.robot_id = robot.id
        self.obstacles = obstacles
        self.poobjects = poobjects
        print(self.obstacles)
        self.camera_link = camera_link
        self.camera_orientation = camera_orientation
        self.state_counter = 0
        for f in os.listdir('./camera'):
            os.remove(os.path.join('./camera', f))

        self.space = PbStateSpace(robot.num_dim)

        bounds = ob.RealVectorBounds(robot.num_dim)
        joint_bounds = self.robot.get_joint_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.space)

        # TODO check camera image
        self.ss.setStateValidityAndTargetChecker(ob.StateValidityCheckerFn(self.is_state_valid), ob.TargetCheckerFn(self.target_found))
        #self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        # self.si.setStateValidityCheckingResolution(0.005)
        # self.collision_fn = pb_utils.get_collision_fn(self.robot_id, self.robot.joint_idx, self.obstacles, [], True, set(),
        #                                                 custom_limits={}, max_distance=0, allow_collision_links=[])

        self.set_obstacles(obstacles)

        # set POObjects
        # pos = np.array([0, 0])
        # len = np.array([0.1, 0.1])
        # self.si.addObject(pos, len)

        self.set_planner("Partial") # RRT by default

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

        # update collision detection
        self.setup_collision_detection(self.robot, self.obstacles)

        self.si.initWorld(len(self.poobjects))

    def add_obstacles(self, obstacle_id):
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        self.obstacles.remove(obstacle_id)

    def is_state_valid(self, state, world):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        # check self-collision
        self.robot.set_state(self.state_to_list(state))
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if utils.pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                # print(get_body_name(body1), get_body_name(body2))
                return False

        # check collision against partially observable objects
        existing_objects = []
        for i, obj in enumerate(world.getStateInt()):
            if obj == 1:
                existing_objects.append(self.poobjects[i])
        po_pairs = list(product(self.moving_bodies, existing_objects))
        for body1, body2 in po_pairs:
            if utils.pairwise_collision(body1, body2):
                return False

        return True

    # process camera image
    # for now returns true/false depending on whether the object was found
    # may be changed later to location
    def target_found(self, state):
        visible_objects = ou.vectorInt()

        # setting the state seems not to be necessary
        # self.robot.set_state(q)

        projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=3.1)

        position = p.getLinkState(self.robot.id, self.camera_link)[4]  # 0/4
        r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, self.camera_link)[5])
        r = np.reshape(r_mat, (-1, 3))
        orientation = np.dot(r, self.camera_orientation).flatten().tolist()
        up = -np.cross(np.array(self.camera_orientation).flatten(), orientation)
        target = [x + y for x, y in zip(position, orientation)]

        viewMatrix = p.computeViewMatrix(
            cameraEyePosition=position,
            cameraTargetPosition=target,
            cameraUpVector=up)

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=224,
            height=224,
            viewMatrix=viewMatrix,
            projectionMatrix=projectionMatrix)

        # expect target to be green and only green object in scene
        target_mask_green = cv2.inRange(rgbImg, (0, 1, 0, 0), (50, 255, 50, 255))
        target_mask_red = cv2.inRange(rgbImg, (1, 0, 0, 0), (255, 50, 50, 255))

        # print('State: {}'.format(self.state_counter))
        #
        # cv2.imwrite('./camera/rgb_{}.jpg'.format(self.state_counter), rgbImg)
        # cv2.imwrite('./camera/mask_{}.jpg'.format(self.state_counter), target_mask)
        self.state_counter += 1

        if cv2.countNonZero(target_mask_green) > 0:
            visible_objects.append(0)
        if cv2.countNonZero(target_mask_red) > 0:
            visible_objects.append(1)

        return visible_objects

    def setup_collision_detection(self, robot, obstacles, self_collisions = True, allow_collision_links = []):
        self.check_link_pairs = utils.get_self_link_pairs(robot.id, robot.joint_idx) if self_collisions else []
        moving_links = frozenset(
            [item for item in utils.get_moving_links(robot.id, robot.joint_idx) if not item in allow_collision_links])
        self.moving_bodies = [(robot.id, moving_links)]
        self.check_body_pairs = list(product(self.moving_bodies, obstacles))

    def set_planner(self, planner_name):
        '''
        Note: Add your planner here!!
        '''
        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        elif planner_name == "Partial":
            self.planner = og.Partial(self.ss.getSpaceInformation())
            print("Using planner Partial.")
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

        self.ss.setPlanner(self.planner)

    def plan_start_goal(self, start, goal, allowed_time = 5.0):#DEFAULT_PLANNING_TIME
        '''
        plan a path to gaol from the given robot start state
        '''
        print("start_planning")
        print(self.planner.params())

        orig_robot_state = self.robot.get_cur_state()

        # set the start and goal states;
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        self.ss.setStartAndGoalStates(s, g)

        # shape = p.getVisualShapeData(self.robot.id)
        # joints = p.getNumJoints(self.robot.id)
        # info = p.getBodyInfo(self.robot.id)
        # for id in range(p.getNumJoints(self.robot.id)):
        #     print(p.getJointInfo(self.robot.id, id))

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        res = False
        all_sol_path_lists = []
        if solved:
            print("Found solution: interpolating into {} segments".format(INTERPOLATE_NUM))
            # print the path to screen

            pdef = self.ss.getProblemDefinition()
            num_solutions = pdef.getSolutionCount()

            for i in range(num_solutions):
                sol_path_geometric = self.ss.getIdxSolutionPath(i)
                sol_path_geometric.interpolate(INTERPOLATE_NUM)
                sol_path_states = sol_path_geometric.getStates()
                sol_path_list = [self.state_to_list(state) for state in sol_path_states]
                all_sol_path_lists.append(sol_path_list)
            # print(len(sol_path_list))
            # print(sol_path_list)

            # TODO check interpolated states
            # for sol_path in sol_path_list:
            #     self.is_state_valid(sol_path)

            res = True
        else:
            print("No solution found")

        # reset robot state
        self.robot.set_state(orig_robot_state)
        return res, all_sol_path_lists

    def plan(self, goal, allowed_time = DEFAULT_PLANNING_TIME):
        '''
        plan a path to gaol from current robot state
        '''
        start = self.robot.get_cur_state()
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def execute(self, path, dynamics=False, camera=False, projectionMatrix=None, linkid=0, camera_orientation=[[1], [0], [0]]):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        '''
        for q in path:
            if dynamics:
                for i in range(self.robot.num_dim):
                    p.setJointMotorControl2(self.robot.id, i, p.POSITION_CONTROL, q[i],force=5 * 240.)
            else:
                self.robot.set_state(q)
                if camera:
                    shape = p.getVisualShapeData(self.robot.id)
                    # p.getAxisAngleFromQuaternion(self.robot.id)
                    # p.getEulerFromQuaternion(self.robot.id)
                    position = p.getLinkState(self.robot.id, linkid)[4]#0/4
                    r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, linkid)[5])
                    r = np.reshape(r_mat, (-1, 3))
                    orientation = np.dot(r, camera_orientation).flatten().tolist()
                    up = -np.cross(np.array(camera_orientation).flatten(), orientation)
                    target = [x + y for x, y in zip(position, orientation)]

                    # position = p.getVisualShapeData(self.robot.id)[-1][5]
                    # r = R.from_quat(p.getVisualShapeData(self.robot.id)[-1][6])
                    # orientation = r.as_euler('zyx', degrees=True)

                    # print('pos: {}'.format(position))
                    # print('ori: {}'.format(orientation))

                    viewMatrix = p.computeViewMatrix(
                        cameraEyePosition=position,
                        cameraTargetPosition=target,
                        cameraUpVector=up)

                    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                        width=224,
                        height=224,
                        viewMatrix=viewMatrix,
                        projectionMatrix=projectionMatrix)
                    # p.getDebugVisualizerCamera()
                    p.removeAllUserDebugItems()
                    p.addUserDebugLine(position, target, lineColorRGB=[1,0,0], lineWidth=5)
            p.stepSimulation()
            # time.sleep(0.001)



    # -------------
    # Configurations
    # ------------

    def set_state_sampler(self, state_sampler):
        self.space.set_state_sampler(state_sampler)

    # -------------
    # Util
    # ------------

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot.num_dim)]