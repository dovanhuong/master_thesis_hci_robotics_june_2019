# Master Thesis of Major HCI & Robotics
This is source code for master's thesis of Huong Do Van in KIST campus under major of HCI - Robotics. <br><br>
The name of thesis is: <br><br><strong><i>"Efficient Reinforcement Learning in Robotics Manipulation Control by mimicking real world environment with simulator"</i></strong>
<br><br>
Under the instruction of <strong><i>Professor Dr. Lee Woosub</i></strong> and <strong><i>Dr. Kim Soonkyum</i></strong> <br>
From Center Medical Robotics at  Korea Institute of Science and Technology <br>
<br>
<strong> Algorithm implemented: Monte-carlo</strong>
<br>
<h4> Results</h4>
<br>
<strong> Mean of success and sum of the reward after 3000 episodes of training</strong><br>
<p align="center">
  <img src="https://github.com/dovanhuong/master_thesis_hci_robotics_june_2019/blob/master/doc/mean_of_success.png" width="350" title="hover text">
  <img src="https://github.com/dovanhuong/master_thesis_hci_robotics_june_2019/blob/master/doc/sum_of_reward.png" width="350" alt="accessibility text">
</p>
<br>
<strong> The result in simulation: </strong>
<br>
<p align="center">
  
[![](http://img.youtube.com/vi/HVbtnGaIi-s/0.jpg)](http://www.youtube.com/watch?v=HVbtnGaIi-s "")

</p>
<br>
<strong> The result in testbed with the hardware: </strong>
<br>
<p align="center">
  
[![](http://img.youtube.com/vi/HdxvACGRTwI/0.jpg)](http://www.youtube.com/watch?v=HdxvACGRTwI "")

</p>
<h1 style="color:blue;"> Instruction to compile source code</h1>
<h2> Neccessary packages and installation</h2> 
- ROS Kinetic full desktop version framework installation instruction at <a href="http://wiki.ros.org/kinetic/Installation/Ubuntu">here</a> and create catkin workspace for project. <br> 
- Install OpenCV library for ROS at <a href="https://www.learnopencv.com/install-opencv3-on-ubuntu/">here</a><br>
- Install Gazebo 7.0 version simulation test and install packages plugin and control with option of install <strong>Pre-Built Debians</strong> at <a href="http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros">here</a><br>
- Command for install <strong>Pre-Built Debians</strong> integrate with  Gazebo: $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control<br>
- Install Driver package for Camera Intel Realsense D435 for developer at <a href="https://github.com/IntelRealSense/realsense-ros#installation-instructions">here</a><br>
- Download latest Realsense SDK 2.0 at <a href="https://github.com/IntelRealSense/librealsense/releases/tag/v2.16.3">here</a><br>
- Install virtual environment in python3 and OpenAI Baselines by follows the command as below: <br>
        <p>$ sudo apt-get install python-pip<br>
	$ sudo apt-get install python3-pip<br>
	$ sudo pip install virtualenv<br>
	$ virtualenv -p python3 [name_virtual_env]<br>
	$ source [name_virtual_env]/bin/activate<br>
	$ pip install ipython<br>
	$ pip install jupyter<br>
	$ sudo apt-get install python-dev python-pip python3-dev python3-pip<br>
	$ sudo -H pip3 install -U pip numpy<br>
	$ pip install numpy scipy matplotlib scikit-image scikit-learn ipython<br>
	$ pip install tensorflow<br>
	$ git clone https://github.com/openai/baselines.git<br>
	$ cd baselines<br>
	$ pip install -e .<br>
	$ pip install rospkg<br>
	$ deactivate <br></p>
- Download and using Pycharm community for linux and developer to execute algorithm inside the virtual environment at <a href="https://www.jetbrains.com/pycharm/download/#section=linux">here</a><br>
  <h2> Step working with simulation task and reinforcement learning algorithms implementation</h2>
  Before execute, add the text in <strong>bash</strong> file: <strong><i>export GAZEBO_MODEL_PATH=~/catkin_ws/src:{GAZEBO_MODEL_PATH}</i></strong><br>
  Convert from .cpp to .py library of ur_kinematic of Universal Robot under python language with <strong><i>swig</i></strong> tool:<br> 
  $ cd catkin_ws/src/universal_robot/ur_kinematics/src/<br>
  $ ls ( in order to check file ur_kin.i)<br>
  $ swig -c++ -python ur_kin.i (noted to check the content and link directory)<br>
  $ python setup.py build_ext --inplace<br>
  <strong>Compile catkin_ws workspace by execute command:</strong><br>
	$ catkin_make<br>
	$ source devel/setup.bash<br>
  <strong> Execute command to launch simulation environment of golf platform</strong><br>
  $ roslaunch ur_gazebo_test2 ur5_golf_platform.launch<br>
  <strong> After that execute Reinforcement learning algorithms under file:</strong> <br>
  <i>q_learning_0612.py </i> For implement Q learning algorithms in golf platform environment.<br>
  <i>ddpg_ur5_20190416.py</i> For implement DDPG learning algorithms in golf platform environment.<br>
  Or you can create by your own algorithms in this environment<br>
  
   <h2> Step working with real test</h2>
   First, you should connect your UR5 with ROS by checking <strong><i>IP address</i></strong> of UR5 at <a href="https://github.com/olinrobotics/irl/wiki/Tutorial:-Setting-up-and-running-the-UR5-Robotic-Arm">here</a><br> 
Second, change the detail of IP address information inside the file: <strong><i>modman_comm/src/ur_comm.cpp</i></strong> at line 32 with the right name of your UR IP address.<br>
Second, execute connect UR5 with ROS by run command:<br>
$ roslaunch modman_comm ur_comm.launch<br>
Third, execute vision system by the command:<br>
$ roslaunch modman_comm modman_vision.launch <br>
Fourth, execute reinforcement learning algorithm by the policy file at diretory: <strong><i>ur_gazebo_test2/experiment/training_results/policy_best.pkl</i></strong><br>
	














