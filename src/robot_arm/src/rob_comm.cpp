#include <rob_comm.h>

using namespace std;

RobComm::RobComm() {
}

RobComm::~RobComm() {
}
//***************************************************************************
void RobComm::SetRightJoints(float * j)
{

    int i;
    for(i=0;i<6;i++)
    {
         Rx_buf_right[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf][%.2lf][%.2lf][%.2lf]", 
              Rx_buf_right[0],Rx_buf_right[1],Rx_buf_right[2],Rx_buf_right[3],Rx_buf_right[4],Rx_buf_right[5]);

	
}
//***************************************************************************
void RobComm::SetLeftJoints(float * j)
{

    int i;
    for(i=0;i<6;i++)
    {
         Rx_buf_left[i] = j[i];
    }
	
	ROS_INFO("I sent [%.2lf][%.2lf][%.2lf][%.2lf][%.2lf][%.2lf]", 
              Rx_buf_left[0],Rx_buf_left[1],Rx_buf_left[2],Rx_buf_left[3],Rx_buf_left[4],Rx_buf_left[5]);
	
}

//***************************************************************************
void RobComm::GetJoints(float * j)
{


}
