#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <formation_control/Formation.h>


class CurrAdapControl {

public:
	int j;


private:

	int i;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "CurrentAdaptiveControl");

	CurrAdapControl currpctrl;

	ros::spin();

	return 0;
}
