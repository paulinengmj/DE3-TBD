## Troubleshooting
There are only 2 issues that may be encountered during normal usage of the butterfly director code. These same errors may be found in both the simulations and when running the code on the real robot. The mitigation steps vary slightly for each of these.
&nbsp;

#### Troubleshooting in simulation
**********
Error in terminal reads

    $ motion plan found but failed during execution

This error occurs when the coordinates of one of the frames being drawn result in the motion plan calculated being out of the acceptable parameters for the robot to execute. To mitigate this problem first check the coordinates in the frames file to ensure none of them is extremely large or small compared to the others. If not then reduce the value of the variable frameScale in pandaLightPaint to reduce the size of the 3D drawing. This will ensure that all coordinates are easily reachable by the robot and the error will not occur.

When this error happens you may need to restart RViz as well as the Gazebo simulation. Additionally, remember that the simulated robot is initially set up at a point of singularity so it is advised to manually set joint values to set the robot up in a better starting position.

![](animFrames)



#### Troubleshooting on the real robot
*********

Error in terminal reads  

    $ motion plan found but failed during execution

or

    $ no motion plan found

This error occurs for the same reason as in the simulation and the same steps should be carries out to fix the issue in code. When this happens the robot will crash meaning that it needs to be reset before executing the code again. To reset the robot ctrl-c all terminal windows on the computer. Unlock the robot then move it to a slightly new position. Lock the robot again and relaunch everything in the terminal. Run the updated code file and observe for any errors.


