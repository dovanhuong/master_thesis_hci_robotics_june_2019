# state of problem
uint8 state
# 0 : ready
# 1 : get goal position & wait for start position
# 2 : get start position & wait for motion
# 3 : motion generated & request modman to move start pose
# 4 : modman under motion to start pose
# 5 : request modman to move end pose
# 6 : modman under motion to end pose
# 7 : modman motion done $ wait for detecting final position
# 8 : final position detected and send it to gym
# 9 : gym finish evalutaion, end of episode
# 100 : initialize the robot
# goal start, final positions of obejct respectively
float64 object_goal_x
float64 object_goal_y
float64 object_start_x
float64 object_start_y
float64 object_final_x
float64 object_final_y
# motion to manipulate object
float64 robot_start_x
float64 robot_start_y
float64 robot_end_x
float64 robot_end_y
# pixel data to plot
int16 goal_pixel_x
int16 goal_pixel_y
int16 start_pixel_x
int16 start_pixel_y


