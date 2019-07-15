# master_thesis_hci_robotics_june_2019
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
- Command for install <strong>Pre-Built Debians integrate with  Gazebo: \$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control<br>
- Install Driver package for Camera Intel Realsense D435 for developer at <a href="https://github.com/IntelRealSense/realsense-ros#installation-instructions">here</a><br>













