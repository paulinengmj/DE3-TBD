## Failed Attempts

#### Simulation Failures

The main failure which occurred in the simulation was that a motion plan could be found but the execution failed. This was a confusing problem to solve as we couldn't figure out why the specified points of the frame would be out of the dexterous workspace of the end effector. Eventually, it was discovered that two people had both added code to take into account the starting position offset for the frame drawing resulting in most of the points being out of the dexterous boundaries of the Panda robot. To mitigate this issue future code that was written with better comments so that different group members understood each other's additions.

Another failure which occurred was towards the begging of the project. The first code that was written attempted to work without the use of a dedicated motion planning algorithm. We thought that it would be possible to calculate a motion plan mathematically by using DH parameters and the Pseudo inverse at 0.02s intervals of time. Although we managed to get the robot arm to move in a roughly circular motion this method was not aware of any obstacles or it's own joint angle limits so would be very difficult to get working on the real robot.

&nbsp;

#### Panda Failures

During early testing, the Panda robot would often become stuck in singularity resulting in the entire set up having to be restarted. This was usually caused by coordinated of frames being chosen outside the dexterous workspace of the robot which was mitigated by scaling down the butterfly to be drawn.

The physical attachment of the custom light stick to the end effector was very difficult and should be improved in the next iteration. The magnetic docking part had to be tapped onto one side of the gripper which provided a week bendy connection which meant the light wobbled around. This could be improved by bolting the magnetic docking connector to the gripper.

Another failure which occurred which was when trying to pick up the lightstick. Because computer vision wasn't used, the lightstick had to be placed in the same location during setup each time which wasted time as it would often fail to pick up the light. 

&nbsp;

#### Photography Failures
Throughout the project, several long exposure photos were taken using different apps and manually using the camera. We discovered that it is very challenging to obtain good quality light painted images from mobile apps, an example of which can be seen below. The amount of background lighting has a huge impact on the quality of the photos taken so it is crucial to keep this to a minimum.

![](butterflyFail.jpg)

