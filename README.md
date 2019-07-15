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
  <h2> Step working with simulation task and reinforcement learning algorithms implementation</h2>
  













