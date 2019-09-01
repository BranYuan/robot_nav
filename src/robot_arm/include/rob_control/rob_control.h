#ifndef rob_control_H
#define rob_control_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>

#include <list>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>//header is the standard action library to implement an action server node

#include <control_msgs/FollowJointTrajectoryAction.h>////The header is generated from the stored action files

#include "rob_comm.h"

struct robotState
{
	float j[5];		// joint position
	float duration;	// duration for motion; needed for actionServer
};

namespace rob_robots
{

	// the struct stores information about the current robots state:
	// joint positions, cartesian position and error code

	class rob_control
	{

		private:

			bool flag_stop_requested;

			robotState r_setPointState;
			robotState r_targetState;
			robotState l_setPointState;
			robotState l_targetState;

			int nrOfJoints; /* 5 */
			double cycleTime; /* in ms */

			//control data def	
			float r_controlData[6];	/* final control data */
			float r_executeData[6];
			float l_controlData[6];	/* final control data */
			float l_executeData[6];
			float r_executeData_old[6];
			float l_executeData_old[6];
			float neck_controlData[2];

			RobComm Comm;

			ros::NodeHandle n;
			sensor_msgs::JointState r_msgJointsCurrent;	/* the current joints */
			sensor_msgs::JointState l_msgJointsCurrent;	/* the current joints */
			ros::Publisher  pubJoints; /* publishes the current joint positions  */

			void MotionGeneration();
			void CommunicationHW();
			void L_CommunicationROS();
			void R_CommunicationROS();
			
			
		public:
			~rob_control();
			void init();
			void mainLoop();				
	};

}

#endif
