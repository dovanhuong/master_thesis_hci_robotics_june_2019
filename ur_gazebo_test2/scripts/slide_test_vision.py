import gym
import gym.spaces
# from openai_ros.task_envs.UR5 import slide_puck
from ur_gazebo_test2.scripts import slide_puck_vision
import rospy
import numpy as np
import time

if __name__ == '__main__':

    rospy.init_node('UR5_slide_test')
    rospy.Time.now()

    env = gym.make('UR5Slide-v2')
    env.env.my_init(True)   # True: use gazebo sim,  False: use skkim's sim
    rospy.loginfo("gym env done")

    env.reset()

    for _ in range(100):
        key_input = input("Press ANY KEY to CONTINUE...")
        action = [0.4, 0.1, 0.0, 0.0]
        # action = [np.random.uniform(0.15, 0.25),
        #           np.random.uniform(-0.05, 0.05),
        #           np.random.uniform(np.radians(-5), np.radians(5)),
        #           np.random.uniform(np.radians(-5), np.radians(5))]
        env.step(action)
        env.reset()