## Getting started
*****
This section details how to connect the robot and test the setup by using FCI to read the current state.

##### Operating the robot
Before going further though, here are a few safety considerations. Always check the following things before powering on the robot:

- Make sure that the arm has been mounted on a stable base and cannot topple over, even when performing fast motions or abrupt stops.

        CAUTION!  Only tabletop mounting is supported, i.e. the Arm must be mounted perpendicular to the ground! Other mountings will void your warranty and might cause damage to the robot!

- Ensure that the cable connecting Arm and Control is firmly attached on both sides.
- Connect the external activation device to Arm’s base and keep it next to you in order to be able to stop the robot at any time.

        HINT!  Activating the external activation device will disconnect the Arm from Control. The joint motor controllers will then hold their current position. The external activation device is not an emergency stop!


        IMPORTANT! The workstation PC which commands your robot using the FCI must always be connected to the LAN port of Control (shop floor network) and not to the LAN port of the Arm (robot network).
**Please read the chapter dedicated to safety in the manual delivered with your robot in addition to this.**

#### Setting up the network
Good network performance is crucial when controlling the robot using FCI. Therefore it is strongly recommended to use a direct connection between the workstation PC and Panda’s Control. This section describes how to configure your network for this use case.

        NOTE! Use Control’s LAN port when controlling the robot through FCI. Do not connect to the port in Arm’s base.

The Control and your workstation must be configured to appear on the same network. Simplest way to achieve that is to use static IP addresses. Any two addresses on the same network would work, but the following values will be used for the purpose of this tutorial:

||Workstation PC	|Control|
|--------------|--------------------|--------------|
|Address	|172.16.0.1	|172.16.0.2|
|Netmask	|24	|24|

        The Control’s address (172.16.0.2) is called ```fci-ip```.


The configuration process consists of two steps:

- Configuring Control’s network settings.
- Configuring your workstation’s network settings.

Which can be found in detailed steps on the franka emika documentation [here](https://frankaemika.github.io/docs/index.html)

Once setting up, you are ready to follow through with the following sections.

#### Franka Emika Software
Software updates can be found [here](http://support.franka.de/)

The software versions currently used in the robotics lab are:

Software | Version | 
--- | --- | 
Franka Firmware  |  1.0.9 | 
Supports libfranka |< 0.2.0 | 
ros-kinetic-libfranka| 0.1.0   | 
Current ROS version   | 0.2.0   | 