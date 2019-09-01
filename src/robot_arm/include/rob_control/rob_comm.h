#ifndef rob_comm_H
#define rob_comm_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <string.h>
#include <sstream>

class RobComm {
public:
	RobComm();
	virtual ~RobComm();

	//bool flagDoComm;
	int nrOfJoints;
	float Rx_buf_right[6];    
    float Rx_buf_left[6];                  
	float Tx_buf[12];

	//void Init(std::string robotType);
	void SetRightJoints(float * j);
    void SetLeftJoints(float * j);
	void GetJoints(float * j);

};

#endif
