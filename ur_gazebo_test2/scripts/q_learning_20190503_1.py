
from ur_gazebo_test2.scripts import slide_puck
from ur_gazebo_test2.scripts.slide_puck import UR5SlidePuckEnv
import rospy
import numpy as np
import time
import universal_robot.ur_kinematics.src.ur_kin_py as ur_kin_py
import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float32

import csv

# ---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))
MAX_EPISODES = 10 #10001
MAX_EPOCHS = 100
MAX_STEPS = 10
MAX_BUFFER = 100000
rewards_all_episodes = []

STATE_DIMENSION = 2
ACTION_DIMENSION = 4


ACTION_L1_MIN = 0.3000
ACTION_L2_MIN = 0.1500
ACTION_THETA1_MIN = np.radians(-8) #-0.0800 #0.08
ACTION_THETA2_MIN = np.radians(-8)

ACTION_L1_MAX = 0.35000   # 0.35
ACTION_L2_MAX = 0.200  # 0.2
ACTION_THETA1_MAX = np.radians(8)
ACTION_THETA2_MAX = np.radians(8)


def set_state():
    x_pixel = np.arange(0.4 - 0.1 / 2, 0.4 + 0.1 / 2, 0.05)
    y_pixel = np.arange(-0.1 - 0.1 / 2, -0.1 + 0.1 / 2, 0.05)
    x = random.choice(x_pixel)
    y = random.choice(y_pixel)
    s_state = (x,y)

    return s_state

def set_action():
    action = [np.random.uniform(ACTION_L1_MIN, ACTION_L1_MAX), np.random.uniform(ACTION_L2_MIN, ACTION_L2_MAX),
              np.random.uniform(ACTION_THETA1_MIN, ACTION_THETA1_MAX),
              np.random.uniform(ACTION_THETA2_MIN, ACTION_THETA2_MAX)]

    return action


if __name__ == '__main__':

    rospy.init_node('UR5_play_golf')
    rospy.Time.now()

    env = UR5SlidePuckEnv()

    env.my_init(True)
    rospy.loginfo("gym env done")

    pub_result = rospy.Publisher('result', Float32, queue_size=5)
    result = Float32()

    start_time = time.time()
    past_action = np.array([0.30, 0.15, -0.08, -0.08])
    csvData = []
    num_success = 0
    state = env.reset()
    #action = [0.0,0.0,0.0,0.0]
    #state_action_good = [(state, action)]
    state_action_good = [()]
    state_action_good_test = [()]
    num_success_training = 0
    num_success_testing = 0
    state_good = []
    action_good = []
    state_good_test = []
    action_good_test = []
    state_good_fin = []
    action_good_fin = []
    print('This is training phase ....')
    seen_state_action_pairs = set()
    seen_state = set()
    state_bad = []
    action_bad = []
    success_record = 0

    with open('./result/Training_result_16.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        csvData = [['action_l1', 'action_l2', 'action_theta1', 'action_theta2','initial_position_x', 'initial_position_y', 'achieved_position_x', 'achieved_position_y','distance', 'reward',
                    'success']]
        writer.writerows(csvData)
    csvFile.close()

    with open('./result/Testing_result_16.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        csvData = [['action_l1', 'action_l2', 'action_theta1', 'action_theta2','initial_position_x', 'initial_position_y', 'achieved_position_x', 'achieved_position_y','distance','reward',
                    'success']]
        writer.writerows(csvData)
    csvFile.close()

    s_state = set_state()


    for epoches in range(MAX_EPOCHS):

    ##### Training phase

        num_success_training = 0
        for ep in range(MAX_EPISODES):
            print('\n')
            print('Training EPISODES: ', ep)
            env = UR5SlidePuckEnv()
            state_init = env.reset()
            env.my_init(True)
            initialposition_x = state_init[0]
            initialposition_y = state_init[1]
            state_save = state_init


            action = set_action()
            #action = [np.random.uniform(ACTION_L1_MIN, ACTION_L1_MAX), np.random.uniform(ACTION_L2_MIN, ACTION_L2_MAX),np.random.uniform(ACTION_THETA1_MIN, ACTION_THETA1_MAX), np.random.uniform(ACTION_THETA2_MIN, ACTION_THETA2_MAX)] # 0.22237569493598391 0.19318792963953946 0.013531360102227913 0.05117139092199888

            state, distance, reward, done, success, info = env.step(action)
            next_state = state

            achieved_position_x = next_state[0]
            achieved_position_y = next_state[1]
            #sa = (state, action)
            state_init = env.reset()


            #state_action_good = [(state, action)]
            if success == True:
                state_good.append(state_save)
                action_good.append(action)
                state_action_good.append((state, action))


                """
                    if (len(state_good) >=1):
                        for i in range(len(state_good)-1):
                            if (state != state_good[i-1]).all():
                                state_good_fin.append(state)
                                action_good_fin.append(action)
                                state_action_good.append((state, action))
                                
                """
                num_success_training += 1
                success_record = 1

            else:
                success_record = 0



            now_success_rate = num_success_training/(ep+1)*100.00
            print('The percentage sucess in training phase until ' , ep, ' is: ', now_success_rate, ' (%)')

            with open('./result/Training_result_16.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                csvData = [[str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                            str(initialposition_y), str(achieved_position_x), str(achieved_position_y),str(distance),str(reward), str(success_record)]]
                writer.writerows(csvData)

            csvFile.close()

            #print(state_good)
            #print(action_good)

        print('Completed training phase for golf putting task!')
        print('\n')
        print('Testing phase ....')


    #### Testing Phase!!!!
        num_success_testing = 0
        for ep in range(MAX_EPISODES):
            print('\n')
            print('Test EPISODES: ', ep)
            state_check = env.reset()
            initialposition_x = state_check[0]
            initialposition_y = state_check[1]
            #if state_check in state_action_good:

            if (len(state_good) >=1):
                for i in range(len(state_good) - 1):

                    while (state_check ==state_good[i-1]).all():

                    #if ((state_check == state_good[i-1]).all()):

                        action = action_good[i-1]
                        #action = action1
                        #next_state, reward, done,success, info = env.step(action)
                        state, distance, reward, done, success, info = env.step(action)
                        #state_check = env.reset()
                        next_state = state

                        achieved_position_x = next_state[0]
                        achieved_position_y = next_state[1]

                        if success == True:
                            state_good.append(state_check)
                            action_good.append(action)
                            state_action_good_test.append((state, action))
                            """
                                if (len(state_good) >= 1):
                                    for i in range(len(state_good_fin) - 1):
                                        if (state != state_good_fin[i - 1]).all():
                                            state_good_fin.append(state)
                                            action_good_fin.append(action)
                                            state_action_good.append((state, action))
                                            
                            """
                            num_success_training += 1
                            success_record = 1
                            with open('./result/Testing_result_16.csv', 'a') as csvFile:
                                writer = csv.writer(csvFile)
                                csvData = [
                                    [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                     str(initialposition_x),
                                     str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                     str(distance), str(reward), str(success_record)]]
                                writer.writerows(csvData)

                            csvFile.close()

                        else:
                            #state_check = env.reset()
                            #del state_good[i-1]
                            #state_good.pop(i-1)
                            #state_good = state_good
                            #del action_good[i-1]
                            #action_good.pop(i-1)
                            #action_good = action_good

                            success_record = 0
                            """
                            with open('./result/Testing_result_3.csv', 'a') as csvFile:
                                writer = csv.writer(csvFile)
                                csvData = [
                                    [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                     str(initialposition_x),
                                     str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                     str(distance), str(reward), str(success_record)]]
                                writer.writerows(csvData)

                            csvFile.close()
                            """

                        state_check = env.reset()

                        #break

                    now_success_rate = num_success_testing / (ep + 1) * 100.00
                    print('The percentage sucess in test phase until ', ep, ' is: ', now_success_rate, ' (%)')
                    """
                    with open('./result/Testing_result_2.csv', 'a') as csvFile:
                        writer = csv.writer(csvFile)
                        csvData = [
                            [str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                             str(initialposition_y), str(achieved_position_x), str(achieved_position_y),str(distance), str(reward), str(success_record)]]
                        writer.writerows(csvData)

                    csvFile.close()
                        #break
                    """
                    #else:
                        #break

            else:
                print(' You need more training to get a good dataset! Please try again with training test')
                #break

            action = set_action()
            #action = [np.random.uniform(ACTION_L1_MIN, ACTION_L1_MAX), np.random.uniform(ACTION_L2_MIN, ACTION_L2_MAX),np.random.uniform(ACTION_THETA1_MIN, ACTION_THETA1_MAX), np.random.uniform(ACTION_THETA2_MIN, ACTION_THETA2_MAX)]
            #next_state, reward, done,success, info = env.step(action)
            state, distance, reward, done, success, info = env.step(action)
            next_state = state
            achieved_position_x = next_state[0]
            achieved_position_y = next_state[1]

            if success == True:

                state_good.append(state_check)
                action_good.append(action)
                state_action_good.append((state, action))
                """
                if (len(state_good) >= 1):
                    for i in range(len(state_good) - 1):
                        if (state != state_good_fin[i - 1]).all():
                            state_good_fin.append(state)
                            action_good_fin.append(action)
                            state_action_good.append((state, action))
                            
                """
                num_success_training += 1
                success_record = 1


            else:
                success_record = 0

            now_success_rate = num_success_testing / (ep + 1) * 100.00
            print('The percentage sucess in test phase until ', ep, ' is: ', now_success_rate, ' (%)')

            with open('./result/Testing_result_16.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                csvData = [
                    [str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                     str(initialposition_y), str(achieved_position_x), str(achieved_position_y),str(distance),str(reward), str(success_record)]]
                writer.writerows(csvData)

            csvFile.close()



    print('Completed training!!!!!!')
