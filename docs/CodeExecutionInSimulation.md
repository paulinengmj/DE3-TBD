## Code Execution in Simulation

This section explains how to run the python code on a Panda Robot Simulation. Before attempting to light paint using a real robot it is recommended to first get it working in the Gazebo simulator within your virtual machine. This allows you to test your custom animations and verify everything is working without wasting time on a real robot.

#### 1. Downloading Code
**********
Once you have completed the previous steps to set up ROS on your computer, with the required modules, you can now use the light painting code in the Gazebo simulator. Up to date files can be found on GitHub at this [link](https://github.com/HarveyU1/LightPaintingRobot).
Download ```pandaLightPaint.py``` and ```frames.py``` onto your Linux virtual machine.


#### 2. Launching Gazebo
**********
Open a new terminal window and divide it into 6 windows and type in the following.

    In terminal 1 type      $roscore
    
    In terminal 2 type      $cd catkin_ws
                            $cd source devel/setup.bash
                            $rosrun gazebo_ros gazebo
    
    In terminal 3 type      $cd catkin_ws
                            $source devel/setup.bash
                            $roslaunch franka_gazebo panda_arm_hand.launch

You should see that the gazebo simulator has launched and a model of the panda robot can be seen pointing upwards in the environment.


#### 3. Launching MoveIt
**********
To launch and set up MoveIt for motion planning do the following

    In terminal 4 type      $cd_catkin_ws
                            $source devel/setup.bash
                            $roslaunch panda_moveit_config demo.launch rviz_tutorial:=true


Once MoveIt has started, add motion planning and change the planning scene topic field to ```/planning_scene```

In planning request, change planning group to ```panda_arm```


#### 4. Light Trails in MoveIt
**********
By tracing the position of the end effector in RViz, it allows the path of the attached light-painting module to be recreated in the RViz viewport which helps when testing new animations. This effectively generates a digital model of the light painting that the Panda robot would recreate in real life.

To initialize the light tracing simulation in RViz, go to:

    1. Motion Planning
    2. Planned Path
    3. Links
    4. Check Panda_Hand[_]
    5. Tick the box Show Trail [_]

When the simulation is running in RViz, you will see a line which shows the trajectory the end effector will move along.


#### 5. Connect MoveIt to Gazebo
**********
To create the node which connects moveIt with Gazebo, type the following

    In terminal 5 type      $cd catkin_ws
                            $cd src/panda_publisher
                            $python panda_publisher.py

#### 6. launching pandaLightPaint
**********
Everything is now set for the simulation so you can launch the ```pandaLightPaint``` python code

    In terminal 6 type      $cd catkin_ws
                            cd to the folder you stored the code in
                            $python pandaLightPaint.py

The first time you run this you will get some error messages as the robot starts at singularity in the simulation. After about a minute it will find a valid motion plan to get out of singularity. At this point cancel the execution using ```ctrl c``` then start the code again. It should now run properly without errors.

You should see the robot drawing individual frames in both gazebo and RViz and pausing in between each frame to allow the photo to be processed.