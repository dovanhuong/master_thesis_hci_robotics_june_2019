import gym
import gym.spaces
from ur_gazebo_test2.scripts import slide_puck
import rospy
import numpy as np
import time
import universal_robot.ur_kinematics.src.ur_kin_py as ur_kin_py
from ur_gazebo_test2.scripts.slide_puck import UR5SlidePuckEnv

if __name__ == '__main__':

    rospy.init_node('UR5_slide_test')
    rospy.Time.now()

    #env = gym.make('UR5Slide-v1')
    #env.env.my_init(True)   # True: use gazebo sim,  False: use skkim's sim
    rospy.loginfo("gym env done")
    #rospy.init_node('UR5_play_golf')
    rospy.Time.now()

    env = UR5SlidePuckEnv()

    env.my_init(True)
    rospy.loginfo("gym env done")

    env.reset()

    for _ in range(5):
        #action = np.array([0.95, 0.95, 0.0, 0.0])
        #action[0] = np.random.uniform(0.3, 0.35)
        #action[1] = np.random.uniform(0.15, 0.2)
        #action[2] = np.random.uniform(np.radians(-3), np.radians(3))
        #action[3] = np.random.uniform(np.radians(-3), np.radians(3))

        action = [np.random.uniform(0.3, 0.4),np.random.uniform(0.15, 0.3),np.random.uniform(np.radians(-5), np.radians(5)),np.random.uniform(np.radians(-5), np.radians(5))]
        #action = [0.33561089732242483, 0.18068488539784278, 0.023570786449588974, -0.012579761875864681]
        #action = [0.335, 0.180, 0.023, -0.012]
        #action = [0.21146460931565408, 0.2936361904494221, 0.034899480069382496, -0.07067575584814231]
        #action = [0.21146460931565408, 0.2936361904494221, 0.034899480069382496, -0.07067575584814231]
        #action = [0.3136726540225021, 0.22885891800435948, -0.07791202206884107, 0.06320638988913695]
        #action = [(0.3136726540225021+0.21146460931565408+0.33561089732242483)/3.0,
                  #(0.22885891800435948+0.18068488539784278+0.2936361904494221)/3.0,
                  #(-0.07791202206884107+0.023570786449588974+0.034899480069382496)/3.0,
                  #(0.06320638988913695+(-0.012579761875864681)+(-0.07067575584814231))/3.0]
        #print(action[0], action[2],action[3])
        env.step(action)



        env.reset()

    # x = np.identity(4)
    # ordered_joint_position = [0, -np.pi, 0, -np.pi, 0, 0]
    # ur_kin_py.forward(ordered_joint_position, x.ravel())