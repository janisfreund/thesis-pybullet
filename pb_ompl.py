import math

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
import utils
import time
from itertools import product
import copy
import numpy as np
import cv2
import os
import operator
from scipy.spatial.transform import Rotation as R

from examples.camera_state_sampler import CameraStateSampler

INTERPOLATE_NUM = 1000
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

class PbCarSpace(ob.ReedsSheppStateSpace):
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
    def __init__(self, robot, obstacles = [], poobjects = [], poobjects_properties = [], camera_link = 10, camera_orientation = [[1], [0], [0]], goal_states = [], space="real") -> None:
        '''
        Args
            robot: A PbOMPLRobot instance.
            obstacles: list of obstacle ids. Optional.
        '''
        self.robot = robot
        self.robot_id = robot.id
        self.obstacles = obstacles
        self.poobjects = poobjects
        self.poobjects_properties = poobjects_properties
        # print(self.obstacles)
        self.camera_link = camera_link
        self.camera_orientation = camera_orientation
        self.goal_states = goal_states
        self.state_counter = 0
        self.counter = 0
        for f in os.listdir('./camera'):
            os.remove(os.path.join('./camera', f))

        self.space_name = space
        if space == "car":
            self.space = PbCarSpace(robot.num_dim)

            bounds = ob.RealVectorBounds(2)
            bounds.setLow(-2)
            bounds.setHigh(6)
        else:
            self.space = PbStateSpace(robot.num_dim)

            bounds = ob.RealVectorBounds(robot.num_dim)
            joint_bounds = self.robot.get_joint_bounds()
            for i, bound in enumerate(joint_bounds):
                # TODO hardcoded for debugging
                if i == 0 or i == 1:
                    bounds.setLow(i, -5.5)
                    bounds.setHigh(i, 3.5)
                else:
                    bounds.setLow(i, bound[0])
                    bounds.setHigh(i, bound[1])

        self.space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.space)

        if self.space_name == "car":
            self.ss.getProblemDefinition().setMode(2)
        else:
            if len(self.goal_states) == 0:
                self.ss.getProblemDefinition().setMode(0)
            else:
                self.ss.getProblemDefinition().setMode(1)

        self.si = self.ss.getSpaceInformation()

        if (space == "car" or len(self.goal_states) == 0):
            self.si.initWorld(len(self.poobjects), False)
        else:
            self.si.initWorld(len(self.poobjects), True)

        self.ss.setStateValidityAndTargetChecker(ob.StateValidityCheckerFn(self.is_state_valid), ob.TargetCheckerFn(self.target_found), self.si.getWorld())
        #self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
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

    def add_obstacles(self, obstacle_id):
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        self.obstacles.remove(obstacle_id)

    def is_state_valid(self, state, world):
        self.counter += 1

        # set poobjects to state of the world
        # self.update_poobjects()

        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        self.robot.set_state(self.state_to_list(state))
        # check self-collision
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                # print("Self-collision!")
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if utils.pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                # print(get_body_name(body1), get_body_name(body2))
                # print("Robot collides with normal object!")
                return False

        # check collision against partially observable objects
        existing_objects = []
        for i, obj in enumerate(world.getStateInt()):
            if obj == 1:
                existing_objects.append(self.poobjects[i])
        po_pairs = list(product(self.moving_bodies, existing_objects))
        for body1, body2 in po_pairs:
            if utils.pairwise_collision(body1, body2):
                # print("Robot collides with poobject!")
                # time.sleep(1)
                return False

        # print("State is valid!")
        return True

    # process camera image
    # for now returns true/false depending on whether the object was found
    # may be changed later to location
    # TODO don't hardcode camera properties
    def target_found(self, state):
        # self.update_poobjects()

        visible_objects = ou.vectorInt()

        # setting the state seems not to be necessary
        # q = []
        # for i in range(self.robot.num_dim):
        #     q.append(state[i])
        # self.robot.set_state(q)

        projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=8)

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
            projectionMatrix=projectionMatrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # expect target to be green and only green object in scene
        target_mask_green = cv2.inRange(rgbImg, (0, 1, 0, 0), (50, 255, 50, 255))
        target_mask_red = cv2.inRange(rgbImg, (1, 0, 0, 0), (255, 50, 50, 255))
        target_mask_blue = cv2.inRange(rgbImg, (0, 0, 1, 0), (50, 50, 255, 255))
        target_mask_black = cv2.inRange(rgbImg, (0, 0, 0, 0), (50, 50, 50, 255))


        # print('State: {}'.format(self.state_counter))
        #
        # cv2.imwrite('./camera/rgb_{}.jpg'.format(self.state_counter), rgbImg)
        # cv2.imwrite('./camera/mask_{}.jpg'.format(self.state_counter), target_mask)
        self.state_counter += 1

        if cv2.countNonZero(target_mask_red) > 0 and len(self.poobjects) > 0:
            visible_objects.append(0)
        elif cv2.countNonZero(target_mask_green) > 0 and len(self.poobjects) > 1:
            visible_objects.append(1)
        elif cv2.countNonZero(target_mask_blue) > 0 and len(self.poobjects) > 2:
            visible_objects.append(2)
        elif cv2.countNonZero(target_mask_black) > 0 and len(self.poobjects) > 3:
            visible_objects.append(3)

        # camera_line_id = p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)
        # time.sleep(5)
        # p.removeUserDebugItem(camera_line_id)

        # cv2.imwrite('./camera/rgb_{}.jpg'.format(self.state_counter), rgbImg)
        return visible_objects

    def update_poobjects(self):
        worldState = self.si.getWorld().getStateInt()
        for i, objectState in enumerate(worldState):
            # object exists
            if objectState == 1:
                # check if object exists
                if self.poobjects[i] == -1:
                    box_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=self.poobjects_properties[i][0],
                                               baseCollisionShapeIndex=self.poobjects_properties[i][1], basePosition=self.poobjects_properties[i][2])
                    self.poobjects[i] = box_id
            else:
                try:
                    p.removeBody(self.poobjects[i])
                    self.poobjects[i] = -1
                except:
                    pass

    def update_poobjects_probability(self, p_world):
        for i, pro in enumerate(self.poobjects_properties):
            if len(pro) == 2:
                old_color = p.getVisualShapeData(pro[0])[0][7]
                new_color = list(old_color)
                new_color[3] = p_world[i]
                p.changeVisualShape(pro[0], -1, rgbaColor=new_color)
            else:
                # TODO why +2 to index
                old_color = p.getVisualShapeData(pro[0] + 2)[0][7]
                new_color = list(old_color)
                new_color[3] = p_world[i]
                p.changeVisualShape(pro[0] + 2, -1, rgbaColor=new_color)

    def sample_good_camera_position(self, obj_pos, base_pos, base_offset, camera_link_id):
        # position is directly above base + offset in z direction
        alt_offset = p.getLinkState(self.robot.id, camera_link_id)[4][2]
        position = [base_pos[0], base_pos[1], alt_offset]
        target = obj_pos
        direction = list(map(operator.sub, target, position))
        direction = direction / np.linalg.norm(direction)
        up = [0, 0, 1]

        xaxis = np.cross(up, direction)
        xaxis = xaxis / np.linalg.norm(xaxis)

        yaxis = np.cross(direction, xaxis)
        yaxis = yaxis / np.linalg.norm(yaxis)

        mat = np.array([xaxis, yaxis, direction]).transpose()

        debug_orientation = np.dot(mat, [[0], [0], [1]]).flatten().tolist()
        debug_point = [x + y for x, y in zip(position, debug_orientation)]

        quaternion_ = R.from_matrix(mat).as_quat()
        quaternion = [quaternion_[0], quaternion_[1], quaternion_[3], quaternion_[2]]
        test_quaternion_z = [0, 0, 1, 0] # up
        test_quaternion_y = [0, 0, 0, 1]
        # euler = R.from_matrix(mat).as_euler()

        # xyz = np.cross(position, target)
        # w = [math.sqrt((len(position) ^ 2) * (len(target) ^ 2)) + np.dot(position, target)]

        # orientation = [*xyz, *w]
        # orientation_norm = orientation / np.linalg.norm(orientation)

        inv = p.calculateInverseKinematics(self.robot.id, camera_link_id, position, targetOrientation=quaternion_, maxNumIterations=100)

        state = [base_pos[0], base_pos[1], 0, inv[0], inv[1], inv[2], inv[3], inv[4], inv[5], inv[6]]
        print(state)

        self.robot.set_state(state)

        position_cam = p.getLinkState(self.robot.id, camera_link_id)[4]
        r_quaternion = p.getLinkState(self.robot.id, camera_link_id)[5]
        r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, camera_link_id)[5])
        r = np.reshape(r_mat, (-1, 3))
        orientation = np.dot(r, [[0], [0], [1]]).flatten().tolist()
        debug_target = [x + y for x, y in zip(position_cam, orientation)]

        p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)
        p.addUserDebugLine(position, debug_point, lineColorRGB=[0, 0, 1], lineWidth=5)
        p.addUserDebugLine(position_cam, debug_target, lineColorRGB=[0, 1, 0], lineWidth=5)

    def setup_collision_detection(self, robot, obstacles, self_collisions = True, allow_collision_links = []):
        self.check_link_pairs = utils.get_self_link_pairs(robot.id, robot.joint_idx) if self_collisions else []
        if len(utils.get_moving_links(robot.id, robot.joint_idx)) < 2:
            moving_links = frozenset([-1, 0])
        else:
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


    def set_state_sampler_name(self, sampler_name):
        if sampler_name == "camera":
            multiple_objects = (self.space_name == "car" or len(self.goal_states) == 0)
            _, _, zstate = p.getLinkState(self.robot.id, self.camera_link)[4]
            camera_sampler = CameraStateSampler(self.si, zstate, self.camera_link, self.robot,
                                                [prop[-1] for prop in self.poobjects_properties], multiple_objects, self.space_name)
            self.set_state_sampler(camera_sampler)

    def plan_start_goal(self, start, goal, allowed_time = DEFAULT_PLANNING_TIME):#DEFAULT_PLANNING_TIME
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
        for goal in self.goal_states:
            self.ss.addGoalState(goal)

        # shape = p.getVisualShapeData(self.robot.id)
        # joints = p.getNumJoints(self.robot.id)
        # info = p.getBodyInfo(self.robot.id)
        # for id in range(p.getNumJoints(self.robot.id)):
        #     print(p.getJointInfo(self.robot.id, id))

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        res = False
        all_sol_path_lists = []
        self.tree_path_lists = []
        self.finished_idx = []
        if solved:
            # print the path to screen

            pdef = self.ss.getProblemDefinition()

            num_solutions = pdef.getSolutionCount()

            max_len = INTERPOLATE_NUM
            print("Found solution: interpolating into {} segments".format(max_len))

            lens = []
            for i in range(num_solutions):
                sol_path_geometric = self.ss.getIdxSolutionPath(i)
                sol_path_states = sol_path_geometric.getStates()
                sol_path_list = [self.state_to_list(state) for state in sol_path_states]
                lens.append(self.calc_path_len(sol_path_list))
                if len(sol_path_list) > max_len:
                    max_len = len(sol_path_list)
                # print("First entry: " + str(sol_path_list[0]))
                # print("Last entry: " + str(sol_path_list[-1]) + "\n")
            seg = max_len / np.max(lens)

            # print("-----------------------")

            for i in range(num_solutions):
                sol_path_geometric = self.ss.getIdxSolutionPath(i)

                sol_path_states = sol_path_geometric.getStates()
                sol_path_list = [self.state_to_list(state) for state in sol_path_states]
                self.tree_path_lists.append(sol_path_list)

                interpolate_num = int(lens[i] * seg)
                if interpolate_num == 0 or interpolate_num < len(sol_path_list):
                    interpolate_num = len(sol_path_list)
                self.finished_idx.append(len(sol_path_list))
                sol_path_geometric.interpolateBase(interpolate_num, 2)
                last_state = sol_path_geometric.getState(interpolate_num - 1)
                # last_state = sol_path_geometric.getStates()[-1]
                for n in range(max_len - interpolate_num):
                    if len(sol_path_geometric.getStates()) < max_len:
                        sol_path_geometric.append(last_state)
                sol_path_states = sol_path_geometric.getStates()
                sol_path_list = [self.state_to_list(state) for state in sol_path_states]
                # print("First entry: " + str(sol_path_list[0]))
                # print("Last entry: " + str(sol_path_list[-1]) + "\n")
                # for sol_state in sol_path_list:
                #     print(sol_state[0:2])
                # print("------------------")
                all_sol_path_lists.append(sol_path_list)
                # print(sol_path_list)
                # print("\n")

            res = True
        else:
            print("No solution found")

        # reset robot state
        self.robot.set_state(orig_robot_state)
        return res, all_sol_path_lists, self.tree_path_lists

    def set_start_goal(self, start, goal):#DEFAULT_PLANNING_TIME
        '''
        plan a path to gaol from the given robot start state
        '''
        print(self.planner.params())

        orig_robot_state = self.robot.get_cur_state()

        # set the start and goal states;
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        self.ss.setStartAndGoalStates(s, g)
        for goal in self.goal_states:
            self.ss.addGoalState(goal)

    def calc_path_len(self, path):
        '''
        Calculate length of base movement
        '''
        path_len = 0
        for i in range(len(path) - 1):
            dist = 0
            for n in range(2):
                diff = path[i][n] - path[i+1][n]
                dist += diff * diff
            path_len += math.sqrt(dist)
        return path_len


    def plan(self, goal, allowed_time = DEFAULT_PLANNING_TIME):
        '''
        plan a path to gaol from current robot state
        '''
        start = self.robot.get_cur_state()
        return self.plan_start_goal(start, goal, allowed_time=allowed_time)

    def plan_with_simplification(self, goal):
        start = self.robot.get_cur_state()
        res, all_sol_path_lists, _ = self.plan_start_goal(start, goal, DEFAULT_PLANNING_TIME)

        print("Now planing with simplification")
        time.sleep(10)
        self.set_planner("Partial")
        self.planner.setPathOptimization(True)
        _, all_sol_path_lists_optimized, _ = self.plan_start_goal(start, goal, DEFAULT_PLANNING_TIME)

        return res, all_sol_path_lists, all_sol_path_lists_optimized

    def execute_all(self, paths, drawPaths, dynamics=False, camera=False, projectionMatrix=None, linkid=0, camera_orientation=[[1], [0], [0]], robots=[], stepParam="", raw_path_param="", sol_line_ids=[], line_id=[]):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        '''
        colors = [[1,0,0], [0,1,0], [0,0,1], [0,0,0], [0.5,0,0], [0,0.5,0], [0,0,0.5], [0.5,0.5,0], [0.5,0,0.5], [0,0.5,0.5], [0, 0, 0]]
        po_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 1]]
        # draw path
        if drawPaths and stepParam == "":
            for i in range(self.ss.getProblemDefinition().getSolutionCount()):
                for n in range(len(self.tree_path_lists[i]) - 1):
                    isSame = False
                    for j in range(self.ss.getProblemDefinition().getSolutionCount()):
                        if i != j and len(self.tree_path_lists[j]) > n and self.tree_path_lists[j][n][0] == self.tree_path_lists[i][n][0] and self.tree_path_lists[j][n][1] == self.tree_path_lists[i][n][1] and self.tree_path_lists[j][n + 1][0] == self.tree_path_lists[i][n + 1][0] and self.tree_path_lists[j][n + 1][1] == self.tree_path_lists[i][n + 1][1]:
                            isSame = True
                    if isSame:
                        sol_line_ids.append(p.addUserDebugLine([self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                       [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1], 0],
                                       lineColorRGB=[0, 0, 0], lineWidth=15))
                    else:
                        sol_line_ids.append(
                            p.addUserDebugLine([self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                               [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1],
                                                0], lineColorRGB=colors[self.belief_to_world(self.si.getWorld().getAllBeliefStates()[self.ss.getProblemDefinition().getSolutionIdx()[i]]) % len(colors)], lineWidth=5))
        already_added = dict()
        for i, state in enumerate(self.ss.getProblemDefinition().getObservationPointStates()):
            observations = []
            for observation in self.ss.getProblemDefinition().getObservationPointObservations()[i]:
                observations.append(observation)
            s = self.state_to_list(state)
            key_state = str(s[0]) + "," + str(s[1])
            if not (key_state in already_added):
                self.add_debug_point([s[0], s[1]], 0.1, po_colors[observations[0]])
                already_added[key_state] = 1
            else:
                self.add_debug_point([s[0], s[1]], 0.1, po_colors[observations[0]])
                already_added[key_state] += 1
        p.removeBody(self.robot_id)
        paths_ = np.moveaxis(paths, 0, 1)
        if stepParam == "":
            stepParam = p.addUserDebugParameter('Step', 0, len(paths_) - 1, 0)
        if raw_path_param == "":
            raw_path_param = p.addUserDebugParameter('PathRaw', -1, len(self.tree_path_lists) - 1, -1)
            line = -1
        else:
            line = int(p.readUserDebugParameter(raw_path_param))
        print("Executing paths for " + str(len(robots)) + " robots.")
        text_ids = [None] * len(robots)
        camera_line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=5)
        for state_idx, q in enumerate(paths_):
            if int(p.readUserDebugParameter(raw_path_param)) != line:
                line = int(p.readUserDebugParameter(raw_path_param))
                if line >= 0:
                    solution = self.ss.getProblemDefinition().getRawSolutions()[line]
                    sol_list = [self.state_to_list(state) for state in solution]
                    for s_id in sol_line_ids:
                        p.removeUserDebugItem(s_id)
                    sol_line_ids = []
                    for l_id in line_id:
                        p.removeUserDebugItem(l_id)
                    line_id = []
                    for n in range(len(solution) - 1):
                        line_id.append(p.addUserDebugLine([sol_list[n][0], sol_list[n][1], 0],
                                           [sol_list[n + 1][0], sol_list[n + 1][1], 0],
                                           lineColorRGB=[0, 0, 0], lineWidth=5))
                    for n in range(len(self.tree_path_lists[line]) - 1):
                        isSame = False
                        for j in range(len(solution) - 1):
                            if sol_list[j][0] == self.tree_path_lists[line][n][0] and sol_list[j][1] == \
                                    self.tree_path_lists[line][n][1] and sol_list[j + 1][0] == \
                                    self.tree_path_lists[line][n + 1][0] and sol_list[j + 1][1] == \
                                    self.tree_path_lists[line][n + 1][1]:
                                isSame = True
                        if isSame:
                            line_id.append(p.addUserDebugLine(
                                [self.tree_path_lists[line][n][0], self.tree_path_lists[line][n][1], 0],
                                [self.tree_path_lists[line][n + 1][0], self.tree_path_lists[line][n + 1][1], 0],
                                lineColorRGB=[1, 0, 0], lineWidth=5))
                        else:
                            line_id.append(p.addUserDebugLine(
                                [self.tree_path_lists[line][n][0], self.tree_path_lists[line][n][1], 0],
                                [self.tree_path_lists[line][n + 1][0], self.tree_path_lists[line][n + 1][1], 0],
                                lineColorRGB=[0, 1, 0], lineWidth=5))
                else:
                    for s_id in sol_line_ids:
                        p.removeUserDebugItem(s_id)
                    sol_line_ids = []
                    for l_id in line_id:
                        p.removeUserDebugItem(l_id)
                    line_id = []
                    if drawPaths:
                        for i in range(self.ss.getProblemDefinition().getSolutionCount()):
                            for n in range(len(self.tree_path_lists[i]) - 1):
                                isSame = False
                                for j in range(self.ss.getProblemDefinition().getSolutionCount()):
                                    if i != j and len(self.tree_path_lists[j]) > n and self.tree_path_lists[j][n][0] == \
                                            self.tree_path_lists[i][n][0] and self.tree_path_lists[j][n][1] == \
                                            self.tree_path_lists[i][n][1] and self.tree_path_lists[j][n + 1][0] == \
                                            self.tree_path_lists[i][n + 1][0] and self.tree_path_lists[j][n + 1][1] == \
                                            self.tree_path_lists[i][n + 1][1]:
                                        isSame = True
                                if isSame:
                                    sol_line_ids.append(p.addUserDebugLine(
                                        [self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                        [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1], 0],
                                        lineColorRGB=[1, 0.72, 0], lineWidth=15))
                                else:
                                    sol_line_ids.append(
                                        p.addUserDebugLine(
                                            [self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                            [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1],
                                             0], lineColorRGB=colors[self.belief_to_world(
                                                self.si.getWorld().getAllBeliefStates()[
                                                    self.ss.getProblemDefinition().getSolutionIdx()[i]]) % len(colors)],
                                            lineWidth=5))
            if int(p.readUserDebugParameter(stepParam)) != 0:
                oldStep = 0
                while True:
                    if int(p.readUserDebugParameter(raw_path_param)) != line:
                        line = int(p.readUserDebugParameter(raw_path_param))
                        if line >= 0:
                            solution = self.ss.getProblemDefinition().getRawSolutions()[line]
                            sol_list = [self.state_to_list(state) for state in solution]
                            for s_id in sol_line_ids:
                                p.removeUserDebugItem(s_id)
                            sol_line_ids = []
                            for l_id in line_id:
                                p.removeUserDebugItem(l_id)
                            line_id = []
                            for n in range(len(solution) - 1):
                                line_id.append(p.addUserDebugLine([sol_list[n][0], sol_list[n][1], 0],
                                                                  [sol_list[n + 1][0], sol_list[n + 1][1], 0],
                                                                  lineColorRGB=[0, 0, 0], lineWidth=5))
                            for n in range(len(self.tree_path_lists[line]) - 1):
                                isSame = False
                                for j in range(len(solution) - 1):
                                    if sol_list[j][0] == self.tree_path_lists[line][n][0] and sol_list[j][1] == self.tree_path_lists[line][n][1] and sol_list[j + 1][0] == self.tree_path_lists[line][n + 1][0] and sol_list[j + 1][1] == self.tree_path_lists[line][n + 1][1]:
                                        isSame = True
                                if isSame:
                                    line_id.append(p.addUserDebugLine(
                                        [self.tree_path_lists[line][n][0], self.tree_path_lists[line][n][1], 0],
                                        [self.tree_path_lists[line][n + 1][0], self.tree_path_lists[line][n + 1][1], 0],
                                        lineColorRGB=[1, 0, 0], lineWidth=5))
                                else:
                                    line_id.append(p.addUserDebugLine(
                                        [self.tree_path_lists[line][n][0], self.tree_path_lists[line][n][1], 0],
                                        [self.tree_path_lists[line][n + 1][0], self.tree_path_lists[line][n + 1][1], 0],
                                        lineColorRGB=[0, 1, 0], lineWidth=5))
                        else:
                            for s_id in sol_line_ids:
                                p.removeUserDebugItem(s_id)
                            sol_line_ids = []
                            for l_id in line_id:
                                p.removeUserDebugItem(l_id)
                            line_id = []
                            if drawPaths:
                                for i in range(self.ss.getProblemDefinition().getSolutionCount()):
                                    for n in range(len(self.tree_path_lists[i]) - 1):
                                        isSame = False
                                        for j in range(self.ss.getProblemDefinition().getSolutionCount()):
                                            if i != j and len(self.tree_path_lists[j]) > n and \
                                                    self.tree_path_lists[j][n][0] == self.tree_path_lists[i][n][0] and \
                                                    self.tree_path_lists[j][n][1] == self.tree_path_lists[i][n][1] and \
                                                    self.tree_path_lists[j][n + 1][0] == self.tree_path_lists[i][n + 1][
                                                0] and self.tree_path_lists[j][n + 1][1] == \
                                                    self.tree_path_lists[i][n + 1][1]:
                                                isSame = True
                                        if isSame:
                                            sol_line_ids.append(p.addUserDebugLine(
                                                [self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                                [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1],
                                                 0],
                                                lineColorRGB=[1, 0.72, 0], lineWidth=15))
                                        else:
                                            sol_line_ids.append(
                                                p.addUserDebugLine(
                                                    [self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
                                                    [self.tree_path_lists[i][n + 1][0],
                                                     self.tree_path_lists[i][n + 1][1],
                                                     0], lineColorRGB=colors[self.belief_to_world(
                                                        self.si.getWorld().getAllBeliefStates()[
                                                            self.ss.getProblemDefinition().getSolutionIdx()[i]]) % len(
                                                        colors)], lineWidth=5))
                    if oldStep != int(p.readUserDebugParameter(stepParam)):
                        for i, robot in enumerate(robots):
                            # if state_idx < self.finished_idx[i]:
                            robot.set_state(paths_[int(p.readUserDebugParameter(stepParam))][i])
                            # self.robot.set_state(paths_[int(p.readUserDebugParameter(stepParam))][i])
                            # if drawPaths:
                            #     p.addUserDebugPoints(pointPositions=[[paths_[int(p.readUserDebugParameter(stepParam))][i][0], paths_[int(p.readUserDebugParameter(stepParam))][i][1], 0]],
                            #                          pointColorsRGB=[colors[i % len(colors)]], pointSize=7.5, lifeTime=0)
                            if i == 0 and camera:
                                position = p.getLinkState(robot.id, linkid)[4]  # 0/4
                                r_mat = p.getMatrixFromQuaternion(p.getLinkState(robot.id, linkid)[5])
                                r = np.reshape(r_mat, (-1, 3))
                                orientation = np.dot(r, camera_orientation).flatten().tolist()
                                up = -np.cross(np.array(camera_orientation).flatten(), orientation)
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

                                # p.removeAllUserDebugItems()
                                p.removeUserDebugItem(camera_line_id)
                                camera_line_id = p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)
                        oldStep = int(p.readUserDebugParameter(stepParam))
                        p.stepSimulation()
            if dynamics:
                for i in range(self.robot.num_dim):
                    p.setJointMotorControl2(self.robot.id, i, p.POSITION_CONTROL, q[i],force=5 * 240.)
            else:
                for i, robot in enumerate(robots):
                    # self.robot.set_state(q[i])
                    # if state_idx < self.finished_idx[i]:
                    robot.set_state(q[i])
                    # print("Robot state: " + self.list_to_string(q[i]))
                    # if drawPaths:
                    #     p.addUserDebugPoints(pointPositions=[[q[i][0], q[i][1], 0]], pointColorsRGB=[colors[i % len(colors)]], pointSize=7.5, lifeTime=0)
                    if text_ids[i] != None:
                        p.removeUserDebugItem(text_ids[i])
                    text_ids[i] = p.addUserDebugText(str(i), [q[i][0] - 0.05, q[i][1] - 0.05, 0.1], [0, 0, 0], 0.2, 0, [ 0, 0, 0, 1 ])
                    if i == 0 and camera:
                        # shape = p.getVisualShapeData(self.robot.id)
                        # p.getAxisAngleFromQuaternion(self.robot.id)
                        # p.getEulerFromQuaternion(self.robot.id)
                        position = p.getLinkState(robot.id, linkid)[4]#0/4
                        r_mat = p.getMatrixFromQuaternion(p.getLinkState(robot.id, linkid)[5])
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
                        # p.removeAllUserDebugItems()
                        p.removeUserDebugItem(camera_line_id)
                        camera_line_id = p.addUserDebugLine(position, target, lineColorRGB=[1,0,0], lineWidth=5)
            p.stepSimulation()
        return stepParam, raw_path_param, sol_line_ids, line_id

    def execute_one_after_another(self, paths, drawPaths, camera=False, projectionMatrix=None, linkid=0, camera_orientation=[[1], [0], [0]]):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        '''
        colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0], [0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5], [0.5, 0.5, 0],
                  [0.5, 0, 0.5], [0, 0.5, 0.5]]
        po_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 1]]
        # draw path
        # if drawPaths:
        #     for i in range(self.ss.getProblemDefinition().getSolutionCount()):
        #         for n in range(len(self.tree_path_lists[i]) - 1):
        #             p.addUserDebugLine([self.tree_path_lists[i][n][0], self.tree_path_lists[i][n][1], 0],
        #                                [self.tree_path_lists[i][n + 1][0], self.tree_path_lists[i][n + 1][1], 0],
        #                                lineColorRGB=colors[i % len(colors)], lineWidth=5)
        params = []
        for i in range(len(paths)):
            params.append(p.addUserDebugParameter('Path ' + str(i), 0, 1, 0))
        camera_line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=5)
        p_worlds = self.ss.getProblemDefinition().getPWorlds()
        path_points = -1
        obs_points = []
        while True:
            for n, path in enumerate(paths):
                print("Executing path " + str(n))
                self.update_poobjects_probability(p_worlds[n])
                print("Current world: " + self.vector_to_string(p_worlds[n]))
                # print path
                if drawPaths:
                    p.removeUserDebugItem(path_points)
                    points = [[point[0], point[1], 0] for point in path]
                    point_colors = [[0, 0, 0] for _ in path]
                    path_points = p.addUserDebugPoints(points, point_colors, 5, 0)
                for item in obs_points:
                    p.removeBody(item)
                for i, state in enumerate(self.ss.getProblemDefinition().getObservationPointStates()):
                    observations = []
                    for observation in self.ss.getProblemDefinition().getObservationPointObservations()[i]:
                        observations.append(observation)
                    s = self.state_to_list(state)
                    if s in path:
                        obs_points.append(self.add_debug_point([s[0], s[1]], 0.1, po_colors[observations[0]]))
                for q in path:
                    oldParam = [0] * len(paths)

                    changed = False
                    changedIdx = 0
                    for idx, param in enumerate(params):
                        if int(p.readUserDebugParameter(param)) > oldParam[idx]:
                            changed = True
                            changedIdx = idx
                        oldParam[idx] = int(p.readUserDebugParameter(param))
                    if changed:
                        self.update_poobjects_probability(p_worlds[changedIdx])
                        if drawPaths:
                            p.removeUserDebugItem(path_points)
                            points = [[point[0], point[1], 0] for point in path]
                            point_colors = [[0, 0, 0] for _ in path]
                            path_points = p.addUserDebugPoints(points, point_colors, 5, 0)
                        for q in paths[changedIdx]:
                            self.robot.set_state(q)

                            if camera:
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

                                width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                                    width=224,
                                    height=224,
                                    viewMatrix=viewMatrix,
                                    projectionMatrix=projectionMatrix)

                                p.removeUserDebugItem(camera_line_id)
                                camera_line_id = p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)

                            p.stepSimulation()
                            time.sleep(0.01)

                    else:
                        self.robot.set_state(q)

                        if camera:
                            position = p.getLinkState(self.robot.id, linkid)[4]#0/4
                            r_mat = p.getMatrixFromQuaternion(p.getLinkState(self.robot.id, linkid)[5])
                            r = np.reshape(r_mat, (-1, 3))
                            orientation = np.dot(r, camera_orientation).flatten().tolist()
                            up = -np.cross(np.array(camera_orientation).flatten(), orientation)
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

                            p.removeUserDebugItem(camera_line_id)
                            camera_line_id = p.addUserDebugLine(position, target, lineColorRGB=[1,0,0], lineWidth=5)
                    p.stepSimulation()
                    time.sleep(0.01)

    def execute(self, path, dynamics=False, camera=False, projectionMatrix=None, linkid=0,
                camera_orientation=[[1], [0], [0]]):
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
                    p.setJointMotorControl2(self.robot.id, i, p.POSITION_CONTROL, q[i], force=5 * 240.)
            else:
                self.robot.set_state(q)
                if camera:
                    shape = p.getVisualShapeData(self.robot.id)
                    # p.getAxisAngleFromQuaternion(self.robot.id)
                    # p.getEulerFromQuaternion(self.robot.id)
                    position = p.getLinkState(self.robot.id, linkid)[4]  # 0/4
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
                    p.addUserDebugLine(position, target, lineColorRGB=[1, 0, 0], lineWidth=5)
            p.stepSimulation()
            # time.sleep(0.001)

    # gets num_paths_per_idx paths per world
    def print_tree(self, paths, num_paths_per_idx, is_belief):
        currIdx = -1
        idxParam = p.addUserDebugParameter('idxSelect', 0, int(len(paths)/num_paths_per_idx)-1, 0)
        while True:
            newIdx = int(p.readUserDebugParameter(idxParam))
            if newIdx != currIdx:
                currIdx = newIdx
                p.removeAllUserDebugItems()
                if is_belief:
                    l = self.si.getWorld().getAllBeliefStates()[currIdx]
                    p.addUserDebugText('Belief: ' + self.list_to_string(l), [-2.5, 2.5, 0], [0, 0, 0], 0.5, 0,
                                       [0, 0, 0, 1])
                    print('\n\n\nWorlds:')
                    for i in range(len(self.si.getWorld().getWorldStates())):
                        w = self.si.getWorld().getStateIntFromObjectState(self.si.getWorld().getWorldStates()[i])
                        print(str(i) + ': ' + self.list_to_string(w))
                else:
                    l = self.si.getWorld().getStateIntFromObjectState(self.si.getWorld().getWorldStates()[currIdx])
                    p.addUserDebugText('World: ' + self.list_to_string(l), [-2.5, 2.5, 0], [0, 0, 0], 0.5, 0, [ 0, 0, 0, 1 ])
                for i in range(currIdx * num_paths_per_idx, currIdx * num_paths_per_idx + num_paths_per_idx):
                    path = paths[i]
                    if len(path) > 1:
                        p.addUserDebugLine([path[0][0], path[0][1], 0], [path[1][0], path[1][1], 0],
                                           lineColorRGB=[1, 0, 0], lineWidth=5)


    def list_to_string(self, l):
        s = '['
        for i, e in enumerate(l):
            s += str(e)
            if i != len(l) - 1:
                s += ', '
        s += ']'
        return s



    # -------------
    # Configurations
    # ------------

    def set_state_sampler(self, state_sampler):
        self.space.set_state_sampler(state_sampler)

    # -------------
    # Util
    # ------------

    def state_to_list(self, state):
        if self.space_name == "car":
            x = state.getX()
            y = state.getY()
            theta = state.getYaw()
            return [x, y, theta]
        else:
            return [state[i] for i in range(self.robot.num_dim)]

    def vector_to_string(self, vec):
        s = "["
        for v in vec:
            s += str(v)
            s += ", "
        s += "]"
        return s

    def add_debug_point(self, pos, radius, color):
        visBoxId = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=0.01, rgbaColor=color)
        return p.createMultiBody(baseMass=0, baseVisualShapeIndex=visBoxId, basePosition=[pos[0], pos[1], 0.005])

    def belief_to_world(self, belief):
        i = 0
        for b in belief:
            if abs(1 - b) < 0.001:
                return i
            i += 1
        return -1
