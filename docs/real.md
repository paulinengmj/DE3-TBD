## Code Execution On Real Panda Robot

This section explains how to run the python code on a real Franka Emika Panda Robot. Before attempting, please follow the setup instructions in the getting started section.


#### Setting Up Panda Robot
**********
First, release the brakes by making sure the external activation device (EAD) is pressed down. Then, in the controller web interface, click to open brakes. A clicking sound will be heard indicating the EAD button needs to be released. After releasing the EAD button, the LED light will turn white, ready to be used. 

**Note** Remember to always have a designated person holding the safety stop to prevent damage to the robot during an error.

#### Controlling Franka & ROS
**********
With ```libfranka``` and ```franka_ros``` installed in the real-time Linux kernel, the following steps should be carried to set up the robot. 

1. Open terminal and type ``` cd franka_ws ```
2. Then, type ``` source /devel/setup.bash ```
3. Split the terminal into 5 terminals.

In the following order, type in these commands in the respective terminals. 

	In terminal 1 type      $roscore

	In terminal 2 type      $roslaunch franka_control franka_control.launch robot_ip:=192.168.0.88 load_gripper:=true

	In terminal 3 type      $roslaunch panda_moveit_config panda_moveit.launchcontroller:=position

	In terminal 4 type      $roslaunch panda_moveit_config moveit_rviz.launch 


#### Executing pandaLightPaint
**********
The system is now ready to run the ```pandaLightPaint``` code

    In terminal 5 navigate to the folder where you sorted the python code and type
        $python pandaLightPaint.py

The robot should begin drawing the frames in the same way as in the simulation. 
Please refer to the other guide sections for further information on physical set up of the robot.
