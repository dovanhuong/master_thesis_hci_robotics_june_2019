
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


ACTION_L1_MIN = 0.2000
ACTION_L2_MIN = 0.1500
ACTION_THETA1_MIN = np.radians(-8) #-0.0800 #0.08
ACTION_THETA2_MIN = np.radians(-8)

ACTION_L1_MAX = 0.35000   # 0.35
ACTION_L2_MAX = 0.2500  # 0.2
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

def compare_state(state_check, state_good, action_good):
    action_test = []
    i_test = []
    for i in range(len(state_good) -1 ):
        if (state_check == state_good[i-1]).all():
            action_test.append(action_good[i-1])
            i_test.append(i-1)

    return action_test, i_test

def random_choice_list(action_test):
    i = []
    for j in range(len(action_test) - 1):
        i.append(j)

    index = random.choice(i)
    return index





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

    with open('./q_learning_result/Training_result_13.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        csvData = [['action_l1', 'action_l2', 'action_theta1', 'action_theta2','initial_position_x', 'initial_position_y', 'achieved_position_x', 'achieved_position_y','distance', 'reward',
                    'success']]
        writer.writerows(csvData)
    csvFile.close()

    with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
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

            state, distance, reward, done, success, info = env.step(action)
            next_state = state

            achieved_position_x = next_state[0]
            achieved_position_y = next_state[1]
            #state_init = env.reset()
            if success == True:
                state_good.append(state_save)
                action_good.append(action)
                state_action_good.append((state, action))

                num_success_training += 1
                success_record = 1
            else:
                success_record = 0

            now_success_rate = num_success_training/(ep+1)*100.00
            print('The percentage sucess in training phase until ' , ep, ' is: ', now_success_rate, ' (%)')

            with open('./q_learning_result/Training_result_13.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                csvData = [[str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                            str(initialposition_y), str(achieved_position_x), str(achieved_position_y),str(distance),str(reward), str(success_record)]]
                writer.writerows(csvData)

            csvFile.close()
            #env.reset()

        print('Completed training phase for golf putting task!')
        print('\n')
        print('Testing phase ....')

        #### Testing Phase!!!!
        num_success_testing = 0
        for ep in range(MAX_EPISODES):
            print('\n')
            print('Test EPISODES: ', ep)
            env = UR5SlidePuckEnv()
            #state_init = env.reset()
            env.my_init(True)
            state_check = env.reset()
            state_in_for = state_check
            initialposition_x = state_check[0]
            initialposition_y = state_check[1]
            i_test = []

            if (len(state_good) == 1):
                if (state_check == state_good[0]).all():
                    action = action_good[0]
                    state, distance, reward, done, success, info = env.step(action)
                    next_state = state
                    achieved_position_x = next_state[0]
                    achieved_position_y = next_state[1]
                    if success == True:
                        success_record = 1
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()
                    else:
                        #del action_good[0]
                        #del state_good[0]
                        success_record = 0
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()
                else:
                    print(' You need more training to get a good dataset! Please try again with training test')
                    print('\n This is collection dataset for reach to good result!')
                    action = set_action()
                    state, distance, reward, done, success, info = env.step(action)
                    next_state = state
                    achieved_position_x = next_state[0]
                    achieved_position_y = next_state[1]

                    if success == True:
                        state_good.append(state_check)
                        action_good.append(action)
                        state_action_good.append((state, action))
                        success_record = 1
                    else:
                        success_record = 0

                    with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                        writer = csv.writer(csvFile)
                        csvData = [
                            [str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                             str(initialposition_y), str(achieved_position_x), str(achieved_position_y), str(distance),
                             str(reward), str(success_record)]]
                        writer.writerows(csvData)
                    csvFile.close()
                    #env.reset()


            elif (len(state_good) > 1):
                action_test, i_test = compare_state(state_check, state_good, action_good)
                if (len(action_test) == 1 ):
                    action = action_test[0]
                    state, distance, reward, done, success, info = env.step(action)
                    next_state = state
                    achieved_position_x = next_state[0]
                    achieved_position_y = next_state[1]
                    if success == True:
                        success_record = 1
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()
                    else:
                        #del action_good[i_test[0]]
                        #del state_good[i_test[0]]
                        success_record = 0
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()

                elif (len(action_test) >1):
                    index = random_choice_list(action_test)
                    j = i_test[index]
                    action = action_test[index]
                    state, distance, reward, done, success, info = env.step(action)
                    next_state = state
                    achieved_position_x = next_state[0]
                    achieved_position_y = next_state[1]
                    if success == True:
                        success_record = 1
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()
                    else:
                        #del action_good[j]
                        #del state_good[j]
                        success_record = 0
                        with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                            writer = csv.writer(csvFile)
                            csvData = [
                                [str(action[0]), str(action[1]), str(action[2]), str(action[3]),
                                 str(initialposition_x),
                                 str(initialposition_y), str(achieved_position_x), str(achieved_position_y),
                                 str(distance), str(reward), str(success_record)]]
                            writer.writerows(csvData)
                        csvFile.close()
                        #env.reset()
                        #break
                        #state_check = env.reset()
                else:
                    print(' You need more training to get a good dataset! Please try again with training test')
                    print('\n This is collection dataset for reach to good result!')
                    action = set_action()
                    state, distance, reward, done, success, info = env.step(action)
                    next_state = state
                    achieved_position_x = next_state[0]
                    achieved_position_y = next_state[1]

                    if success == True:
                        state_good.append(state_check)
                        action_good.append(action)
                        state_action_good.append((state, action))
                        success_record = 1
                    else:
                        success_record = 0

                    with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                        writer = csv.writer(csvFile)
                        csvData = [
                            [str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                             str(initialposition_y), str(achieved_position_x), str(achieved_position_y), str(distance),
                             str(reward), str(success_record)]]
                        writer.writerows(csvData)
                    csvFile.close()
                    #env.reset()

                    #break
                    #next_state = env.reset()

            else:
                print(' You need more training to get a good dataset! Please try again with training test')
                print('\n This is collection dataset for reach to good result!')
                action = set_action()
                state, distance, reward, done, success, info = env.step(action)
                next_state = state
                achieved_position_x = next_state[0]
                achieved_position_y = next_state[1]

                if success == True:
                    state_good.append(state_check)
                    action_good.append(action)
                    state_action_good.append((state, action))
                    success_record = 1
                else:
                    success_record = 0

                with open('./q_learning_result/Testing_result_13.csv', 'a') as csvFile:
                    writer = csv.writer(csvFile)
                    csvData = [
                        [str(action[0]), str(action[1]), str(action[2]), str(action[3]), str(initialposition_x),
                         str(initialposition_y), str(achieved_position_x), str(achieved_position_y),str(distance),str(reward), str(success_record)]]
                    writer.writerows(csvData)
                csvFile.close()
                #env.reset()
                #next_state = env.reset()
    print('Completed training!!!!!!')
