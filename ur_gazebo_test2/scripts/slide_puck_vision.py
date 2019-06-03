import gym
import rospy
import numpy as np
from gym import spaces
from ur_gazebo_test2.scripts import ur5_env

from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
from gazebo_msgs.msg import ModelStates

import sys
import universal_robot.ur_kinematics.src.ur_kin_py as ur_kin_py

from puck_sim2.srv import *
from gazebo_msgs.srv import GetWorldProperties
from modman_comm.msg import modman_state

import datetime


timestep_limit_per_episode = 1

register(
    id='UR5Slide-v2',
    entry_point = 'ur_gazebo_test2.scripts.slide_puck_vision:UR5SlidePuckEnv',
    timestep_limit = timestep_limit_per_episode,
)

def best_sol(sols, q_guess, weights):
    valid_sols = []
    for sol in sols:
        test_sol = np.ones(6)*9999.
        for i in range(6):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (abs(test_ang) <= 2.*np.pi and
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol)
    if len(valid_sols) == 0:
        return None
    best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
    return valid_sols[best_sol_ind]


def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


def rotation(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R


class Param(object):
    pass


def puck_sim_client(obj_init, hand_init, hand_fin):
    rospy.wait_for_service('puck_sim')
    try:
        puck_sim_handle = rospy.ServiceProxy('puck_sim', puck_sim)
        ret = puck_sim_handle(obj_init, hand_init, hand_fin)
        return ret.obj_fin
    except rospy.ServiceException as e:
        print("service failed: %s", e)


class UR5SlidePuckEnv(ur5_env.UR5Env):
    def __init__(self):

        """
        Make UR5 learn how to slide a puck.
        """

        # We execute this one before because there are some functions that this
        # TaskEnv uses that use variables from the parent class, like the effort limit fetch.
        super(UR5SlidePuckEnv, self).__init__()

        # Here we will add any init functions prior to starting the MyRobotEnv

        # Only variable needed to be set here

        global max_x
        global max_y
        max_x = 0.0
        max_y = 0.0

        rospy.logdebug("Start UR5SlidePuckEnv INIT...")
        number_actions = 4
        # self.action_space = spaces.Box(-1., 1., shape=(number_actions,), dtype='float32')
        # action space: l, r, theta1, theta2
        self.action_space = spaces.Box(low=np.array([0.35, 0.4, np.radians(-3), np.radians(-3)]),
                                       high=np.array([0.25, 0.3, np.radians(3), np.radians(3)]), dtype='float32')
        # self.action_space = spaces.Discrete(number_actions)

        self._is_action_done = False
        self._has_object = True
        self.param = Param()
        # setattr(self.param, 'init_obj_center', 'init_obj_width', 'target_width', 'target_center', 'target_position')
        # setattr(self.param.init_obj_center, 'x', 'y')
        # setattr(self.param.target_center, 'x', 'y')
        # setattr(self.param.target_position, 'x', 'y')
        setattr(self.param, 'init_obj_center', Param())
        setattr(self.param, 'obj_center', Param())
        setattr(self.param, 'target_center', Param())

        setattr(self.param, 'target_position', Param())
        setattr(self.param, 'final_position', Param())

        setattr(self.param.init_obj_center, 'x', 0.43) # used 0.65
        setattr(self.param.init_obj_center, 'y', 0.0)
        setattr(self.param, 'init_obj_width', 0.1)
        setattr(self.param.obj_center, 'x', self.param.init_obj_center.x)
        setattr(self.param.obj_center, 'y', self.param.init_obj_center.y)

        setattr(self.param.target_center, 'x', self.param.init_obj_center.x)
        setattr(self.param.target_center, 'y', self.param.init_obj_center.y + 1.0380) # used 0.5
        setattr(self.param, 'target_width', 0.2)

        setattr(self.param.target_position, 'x', self.param.target_center.x)
        setattr(self.param.target_position, 'y', self.param.target_center.y)
        setattr(self.param.final_position, 'x', self.param.obj_center.x)
        setattr(self.param.final_position, 'y', self.param.obj_center.y)


        temp = SetModelStateRequest()
        self.my_object = temp.model_state.pose
        self.my_target = temp.model_state.pose
        self.my_object.position.x = self.param.init_obj_center.x
        self.my_object.position.y = self.param.init_obj_center.y
        self.my_target.position.x = self.param.target_position.x
        self.my_target.position.y = self.param.target_position.y
        # self.my_object = Param()
        # setattr(self.my_object.)
        # setattr(self.my_object, 'x', self.param.init_obj_center.x)
        # setattr(self.my_object, 'y', self.param.init_obj_center.y)
        # self.my_target = Param()
        # setattr(self.my_target, 'x', self.param.target_position.x)
        # setattr(self.my_target, 'y', self.param.target_position.y)
        obs = self._get_obs()   # get observation
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype='float32'),
        ))

        self.init_joints_positions_array = [-np.pi/2, -np.pi/3, np.pi/3*2, -np.pi/3, 0., 0.]

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._object_state_callback)
        rospy.Subscriber("/drl_state", modman_state, self._visual_object_state_callback)
        self.action_pub = rospy.Publisher("/drl_actions", modman_state, queue_size=1)

        self.modman_state = modman_state()
        self.action_set = modman_state()
        self.action_set.robot_start_x = -100
        self.action_set.robot_start_y = -100
        self.action_set.robot_end_x = -100
        self.action_set.robot_end_y = -100
        self.action_set.state = 0  # 3 : motion generated & request modman to move start pose

        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo("set model_state available")
        self.set_obj_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_world_state = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

        self.cumulated_steps = 0.0
        self.dynamics_param = True
        self.distance_threshold = 0.10
        # self.reward_type = 'sparse'
        self.reward_type = 'sparse'

        self.save_test_log = True
        if self.save_test_log:
            self.log_file = "./log_" + str(datetime.datetime.now()) + ".txt"
            data = "init_x, init_y, target_x, target_y, result_x, result_y, distance\n"
            self.fd = open(self.log_file, 'w')
            self.fd.write(data)
            self.fd.close()

    def my_init(self, dynamics):
        self.dynamics_param = dynamics

    def _set_init_pose(self):

        diff = 100
        while diff >= 1e-1:
            self.joints = []
            self.joints.append(self.init_joints_positions_array)
            self.move_joints_to_angle_blocking(self.joints, 3.0)

            ordered_joint_position = self._get_ordered_joint_attr('position')
            diff = np.linalg.norm(np.array(ordered_joint_position) - np.array(self.init_joints_positions_array))

        return True

    def _set_init_obj_pose(self):
        """
        This function is called 'at _reset_sim() in robot_gazebo_env.py'
        :return:
        """

        self._init_target()
        rospy.loginfo("init target done")

        # while self.modman_state.state != 2:
        #     rospy.sleep(rospy.Duration(0.1))
        #     rospy.loginfo("waiting for state 2. Modman state: " + str(self.modman_state.state))
        #     pass

        self.param.obj_center.x = -self.modman_state.object_start_x
        self.param.obj_center.y = -self.modman_state.object_start_y


        """
        For not using vision
        self.param.obj_center.x = np.random.uniform(self.param.init_obj_center.x - self.param.init_obj_width/2,
                                                    self.param.init_obj_center.x + self.param.init_obj_width/2)
        self.param.obj_center.y = np.random.uniform(self.param.init_obj_center.y - self.param.init_obj_width / 2,
                                                    self.param.init_obj_center.y + self.param.init_obj_width / 2)
        """


        req_position = np.array([self.param.obj_center.x, self.param.obj_center.y, 0.05])
        req_name = 'my_object'
        while not self._set_obj_position(req_name, req_position):
            pass
        rospy.loginfo("init object done")

        req_name = 'my_visual_object'
        req_position = np.array([self.param.obj_center.x, 2.0, 0.05])
        while not self._set_obj_position(req_name, req_position):
            pass
        rospy.loginfo("init visual object done")

        self.my_object.position.x = self.param.obj_center.x
        self.my_object.position.y = self.param.obj_center.y

        return True

    def _init_env_variables(self):
        """
        This function is called right after _reset_sim() in robot_gazebo_env.py
        Thus, we know object's initialized position before this function call.
        :return:
        """
        # For Info Purposes
        self.cumulated_stpes = 0.0
        # self._set_init_obj_pose()
        # self._init_target()
        # self._init_target()
        # TODO: set self.target_position to rosparam server for rviz visualization

    def _init_target(self):

        while self.modman_state.state != 2:
            rospy.sleep(rospy.Duration(0.2))
            rospy.loginfo("waiting for state 2. Modman state: " + str(self.modman_state.state))
            pass

        self.param.target_position.x = -self.modman_state.object_goal_x
        self.param.target_position.y = -self.modman_state.object_goal_y
        """
        
        self.param.target_position.x = np.random.uniform(self.param.target_center.x - self.param.target_width/2,
                                                         self.param.target_center.x + self.param.target_width/2)
        self.param.target_position.y = np.random.uniform(self.param.target_center.y - self.param.target_width/2,
                                                         self.param.target_center.y + self.param.target_width/2)
 
        """

        req_position = np.array([self.param.target_position.x, self.param.target_position.y, 0.05])
        req_name = 'my_target'
        self._set_obj_position(req_name, req_position)

        self.my_target.position.x = self.param.target_position.x
        self.my_target.position.y = self.param.target_position.y
        # target_position = self.my_target.position
        self.goal = np.array([self.my_target.position.x, self.my_target.position.y])

        return True

    def _set_obj_position(self, obj_name, position):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.loginfo("set model_state for " + obj_name + " available")
        sms_req = SetModelStateRequest()
        sms_req.model_state.pose.position.x = position[0]
        sms_req.model_state.pose.position.y = position[1]
        sms_req.model_state.pose.position.z = position[2]
        sms_req.model_state.pose.orientation.x = 0.
        sms_req.model_state.pose.orientation.y = 0.
        sms_req.model_state.pose.orientation.z = 0.
        sms_req.model_state.pose.orientation.w = 1.

        sms_req.model_state.twist.linear.x = 0.
        sms_req.model_state.twist.linear.y = 0.
        sms_req.model_state.twist.linear.z = 0.
        sms_req.model_state.twist.angular.x = 0.
        sms_req.model_state.twist.angular.y = 0.
        sms_req.model_state.twist.angular.z = 0.
        sms_req.model_state.model_name = obj_name
        sms_req.model_state.reference_frame = 'world'
        result = self.set_obj_state(sms_req)

        return result.success



    def _set_action(self, action):
        """
        Fetch robot actions
        :param action:
        :return:
        """
        # l = np.random.uniform(0.15, 0.25)
        # r = np.random.uniform(-0.05, 0.05)
        # r_hit = np.random.uniform(0, 0.05)
        # theta1 = np.random.uniform(-5, 5) /180*np.pi
        # theta2 = np.random.uniform(-5, 5) /180*np.pi
        l = action[0]
        r = action[1]
        theta1 = action[2]
        theta2 = action[3]
        self._is_action_done = False

        # point target
        p_t = np.array([self.param.target_position.x, self.param.target_position.y])
        # point puck
        p_p = np.array([self.param.obj_center.x, self.param.obj_center.y])
        v = (p_t - p_p)/np.linalg.norm(p_t - p_p, 2)
        R1 = rotation(theta1)
        R2 = rotation(theta2)

        # action end position
        p_e = p_p + R2.dot(r*v)

        # action start position
        p_s = p_p - R1.dot(l*v)
        v2 = (p_e - p_s)/np.linalg.norm(p_e - p_s, 2)


        if self.modman_state.state == 2:
            p_e2 = p_e + 0.0*v2
            self.action_set.robot_start_x = -p_s[0]
            self.action_set.robot_start_y = -p_s[1]
            self.action_set.robot_end_x = -p_e2[0]
            self.action_set.robot_end_y = -p_e2[1]
            self.action_set.state = 3        # 3 : motion generated & request modman to move start pose
            self.action_pub.publish(self.action_set)
        else:
            self.action_set.state = 0

        # test
        # v = np.array([0., 1.0])
        # p_s = np.array([0.65, 0.])
        # v2 = v

        # inverse kin for initial position
        ordered_joint_position = self._get_ordered_joint_attr('position')  # get current joint position
        x = np.identity(4)
        ur_kin_py.forward(ordered_joint_position, x.ravel())
        # set desired position
        x[:2, 3] = p_s
        x[2,3] = -0.05
        # set desired orientation
        x_hat = np.append(v2, [0.])
        y_hat = np.cross(x[:3,2], x_hat)
        x[:3, 0] = x_hat
        x[:3, 1] = y_hat
        sols = np.zeros([8,6])
        num_sol = ur_kin_py.inverse(x.ravel(), sols.ravel(), float(0.0))
        if num_sol == 0:
            self._is_action_done = True
            return

        qsol = best_sol(sols, ordered_joint_position, [1.]*6)

        # add to joint list
        joint_list = []
        joint_list.append(qsol.tolist())
        # joint_list.append(qsol.tolist())
        # joint_list.append(qsol.tolist())
        self.move_joints_to_angle_blocking(joint_list, 2.0)
        rospy.loginfo("prepared to action")


        # inverse kin for final position
        xf = x
        N = 1
        joint_list = []
        del_p = p_e - p_s
        for i in range(N):
            p_n = p_s + ((i+1)/N)*del_p
            xf[:2, 3] = p_n
        # xf[2, 3] = -0.1
            sols_f = np.zeros([8,6])
            ur_kin_py.inverse(xf.ravel(), sols_f.ravel(), float(0.0))
            qsol_f = best_sol(sols_f, qsol, [1.]*6)

            # add to joint list
            joint_list.append(qsol_f.tolist())

        # move object to puck_sim result position
        if self.dynamics_param is False:
            ret_pucksim = puck_sim_client(p_p, p_s, p_e)
            result_position = np.append((np.array(ret_pucksim)), 0.05)
            # ret_sop = self._set_obj_position('my_object', result_position)
            while not self._set_obj_position('my_object', result_position):
                pass

        # move
        timestep = 0.5
        init_time = self.get_world_state().sim_time
        self.move_joints_to_angle_blocking(joint_list, timestep)
        self._is_action_done = True

        # wait for seconds
        wait_count = 0
        max_wait_count = 10    # 0.5 * N seconds (default time step is 0.5 sec)
        while self._calc_object_velocity() > 1e-2 and wait_count < max_wait_count:
            joint_list = []
            joint_list.append(qsol_f.tolist())
            self.move_joints_to_angle_blocking(joint_list)
            wait_count += 1


        # self._set_obj_position('my_object', result_position)
        # wait 1 second
        global max_x
        global max_y
        joint_list = []
        joint_list.append(qsol_f.tolist())
        joint_list.append(qsol_f.tolist())
        self.move_joints_to_angle_blocking(joint_list)

        while self.modman_state.state != 8:
            rospy.loginfo("Waiting for final position")
            rospy.sleep(rospy.Duration(0.5))

        if self.modman_state.state == 8:
            #self.param.final_position.x = -self.modman_state.object_final_x
            #self.param.final_position.y = -self.modman_state.object_final_y
            self.param.final_position.x = -max_x
            self.param.final_position.y = -max_y
            req_position = np.array([self.param.final_position.x, self.param.final_position.y, 0.05])
            req_name = 'my_visual_object'
            self._set_obj_position(req_name, req_position)

            ach = np.array([self.param.final_position.x, self.param.final_position.y])
            des = np.array([self.param.target_position.x, self.param.target_position.y])
            rospy.loginfo("object position: " + str(self.param.final_position.x) + ", " + str(self.param.final_position.y))
            result_distance = goal_distance(ach, des)
            rospy.loginfo("distance: " + str(result_distance))

            rospy.loginfo(str(self.param.obj_center.x) + ", " + str(self.param.obj_center.y) + ", "
                          + str(self.param.target_position.x) + ", " + str(self.param.target_position.y) + ", "
                          + str(self.param.final_position.x) + ", " + str(self.param.final_position.y) + ", "
                          + str(result_distance))

            if hasattr(self, 'log_file') and self.save_test_log:
                data = "%f, %f, %f, %f, %f, %f, %f\n" % (self.param.obj_center.x, self.param.obj_center.y,
                                                         self.param.target_position.x, self.param.target_position.y,
                                                         self.param.final_position.x, self.param.final_position.y,
                                                         result_distance)
                self.fd = open(self.log_file, 'a')
                self.fd.write(data)
                self.fd.close()

            rospy.sleep(rospy.Duration(3.0))
            key_input = input("Press any key to reset...")
            self.action_set.state = 9
            self.action_pub.publish(self.action_set)


        # assert action.shape == (4,)
        # action = action.copy()  # ensure that we don't change the action outside of this scope
        # pos_ctrl, gripper_ctrl = action[:3], action[3]
        #
        # pos_ctrl *= 0.05  # limit maximum change in position
        # rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion
        # gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        # assert gripper_ctrl.shape == (2,)
        # if self.block_gripper:
        #     gripper_ctrl = np.zeros_like(gripper_ctrl)
        # action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])
        #
        # # Apply action to simulation.
        # utils.ctrl_set_action(self.sim, action)
        # utils.mocap_set_action(self.sim, action)

        """
        IRI WAM robot actions
        """
        # if action == 0:  # Increase joint_0
        #     self.joints[0] += self.joint_increment_value
        # elif action == 1:  # Decrease joint_0
        #     self.joints[0] -= self.joint_increment_value
        # elif action == 2:  # Increase joint_1
        #     self.joints[1] += self.joint_increment_value
        # elif action == 3:  # Decrease joint_1
        #     self.joints[1] -= self.joint_increment_value
        # elif action == 4:  # Increase joint_2
        #     self.joints[2] += self.joint_increment_value
        # elif action == 5:  # Decrease joint_2
        #     self.joints[2] -= self.joint_increment_value
        # elif action == 6:  # Increase joint_3
        #     self.joints[3] += self.joint_increment_value
        # elif action == 7:  # Decrease joint_3
        #     self.joints[3] -= self.joint_increment_value
        # elif action == 8:  # Increase joint_4
        #     self.joints[4] += self.joint_increment_value
        # elif action == 9:  # Decrease joint_4
        #     self.joints[4] -= self.joint_increment_value
        # elif action == 10:  # Increase joint_5
        #     self.joints[5] += self.joint_increment_value
        # elif action == 11:  # Decrease joint_5
        #     self.joints[5] -= self.joint_increment_value
        # elif action == 12:  # Increase joint_6
        #     self.joints[6] += self.joint_increment_value
        # elif action == 13:  # Decrease joint_6
        #     self.joints[6] -= self.joint_increment_value
        # elif action == 14:  # Increase joint_7
        #     self.joints[7] += self.joint_increment_value
        # elif action == 15:  # Decrease joint_7
        #     self.joints[7] -= self.joint_increment_value
        #
        # # We tell iriwam the action to perform
        # self.move_joints_to_angle_blocking(self.joints)

    def _is_done(self, observations):
        """
        consider the episode done if:
            1)  the object & the robot stops
            2)  error occurs when robot moves
        :param observations:
        :return:
        """
        if self._is_action_done is True and self._calc_object_velocity() < 1e-2\
                and self.modman_state.state == 8:
            done = True
            # ach = np.array([self.param.final_position.x, self.param.final_position.y])
            # des = np.array([self.param.target_position.x, self.param.target_position.y])
            # result_distance = goal_distance(ach, des)
            # if hasattr(self, 'log_file'):
            #     data = "%f, %f, %f, %f, %f, %f, %f\n" % (self.param.obj_center.x, self.param.obj_center.y,
            #                                              self.param.target_position.x, self.param.target_position.y,
            #                                              self.param.final_position.x, self.param.final_position.y,
            #                                              result_distance)
            #     self.fd = open(self.log_file, 'a')
            #     self.fd.write(data)
            #     self.fd.close()
        else:
            done = False
        return done

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        # initial_pos = np.array([self.my_object.position.x, self.my_object.position.y])
        # d_init = goal_distance(initial_pos, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _compute_reward(self, observations, done):
        d = goal_distance(observations['achieved_goal'], observations['desired_goal'])

        # if done:
        #     reward = 1
        # else:
        #     reward = -1

        return -1*d*d

    def compute_reward(self, achieved_goal, desired_goal, info):
        d = goal_distance(achieved_goal, desired_goal)

        if self.reward_type == 'sparse':
            return -(d > self.distance_threshold).astype(np.float32)
        else:
            return -d

    def _get_obs(self):
        """

        Here we define what sensor data defines our robots observations
        :return:
        """
        global max_x
        global max_y

        T = np.identity(4)      # base to robot end.
        ordered_joint_position = self._get_ordered_joint_attr('position')
        ur_kin_py.forward(ordered_joint_position, T.ravel())
        self.get_end_effector_pose(T)   # get SE3 global fixed frame to robot end. not to end effector
        obs = np.concatenate([ordered_joint_position, self._end_eff_pose.ravel()])

        # while not self.my_object:
        #     pass

        # ach = np.array([self.my_object.position.x, self.my_object.position.y])        # simulation result
        #ach = np.array([self.param.final_position.x, self.param.final_position.y])
        ach = np.array(max_x, max_y)

        # real world result
        # ach = np.array([0, 0])
        des = np.array([self.param.target_position.x, self.param.target_position.y])
        # des = [0, 0]
        result_distance = goal_distance(ach, des)
        rospy.loginfo("distance: " + str(result_distance))

        # if hasattr(self, 'log_file'):
        #     data = "%f, %f, %f, %f, %f, %f, %f\n" % (self.param.obj_center.x, self.param.obj_center.y,
        #                                          self.param.target_position.x, self.param.target_position.y,
        #                                          self.my_object.position.x, self.my_object.position.y,
        #                                              result_distance)
        #     self.fd = open(self.log_file, 'a')
        #     self.fd.write(data)
        #     self.fd.close()



        return {
            'observation': obs.copy(),
            'achieved_goal': ach.copy(),
            'desired_goal': des.copy(),
        }

    def _object_state_callback(self, data):
        self.object_states = data
        obj_index = self.object_states.name.index('my_object')
        self.my_object = self.object_states.pose[obj_index]
        self.my_object_linear_vel = self.object_states.twist[obj_index].linear
        self.my_object_angular_vel = self.object_states.twist[obj_index].angular
        # rospy.loginfo("x: " + str(self.my_object.position.x))
        # rospy.loginfo("y: " + str(self.my_object.position.y))
        # rospy.loginfo("z: " + str(self.my_object.position.z))

        target_index = self.object_states.name.index('my_target')
        self.my_target = self.object_states.pose[target_index]

    def _visual_object_state_callback(self, data):
        self.modman_state = data
        global max_x
        global max_y

        if (abs(max_x) <= abs(self.param.modman_state.object_final_x)):
            max_x = self.param.modman_state.object_final_x

        if (abs(max_y) <= abs(self.param.modman_state.object_final_y)):
            max_y = self.param.modman_state.object_final_y


        self.action_pub.publish(self.action_set)

        # pass

    def _calc_object_velocity(self):
        vel = np.linalg.norm([self.my_object_linear_vel.x, self.my_object_linear_vel.y, self.my_object_linear_vel.z])
        return vel













def main():
    rospy.init_node('ur5_slide_puck')

    sys.path.append(" /home/modmanvision/catkin_ws/src/openai_ros/openai_ros/src/openai_ros/task_envs/UR5")
    env = gym.make('UR5Slide-v0')
    rospy.loginfo("GYM environment done")




if __name__ == '__main__':
    main()





