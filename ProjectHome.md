GraspPlugin for Choreonoid provids several functions for robotic systems having multi-fingered hand attached at the tip of a robotic arm. These functions are developed as plug-in functions of Choreonoid.  The following plug-in functions are included:

Grasp planning plug-in: Grasp planning function and many functions commonly used by other plug-ins.

&lt;BR&gt;


Trajectory planning plug-in: Trajectory planning function connecting the start and the goal configurations with avoiding the collision with the obstacles. 

&lt;BR&gt;


Gripper manipulation plug-in: Pick-and-place planning function for robot manipulators having a parallel gripper at the tip. 

&lt;BR&gt;


Vision trigger plug-in: OpenRTM interface in connection with OpenVGR.

&lt;BR&gt;


Robot interface plug-in: OpenRTM interface sending commands to HiroNX dual arm robot.

Here, we ported the Motion Planning Kit (MPK) developed in Stanford university as a trajectory planning plug-in. Prof. Jean-Claude Latombe is kind enough to allow us to release graspPlugin with including MPK. As for the license of the trajectory planning algorithm part, please follow that of MPK. If you download this software, we would be happy if you send an e-mail describing the information on the name of the user to nedo-vms-contact[at](at.md)m.aist.go.jp.