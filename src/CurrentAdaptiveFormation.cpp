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
#include <geometry_msgs/TwistStamped.h>


class CurrAdapControl {

public:

	void init() {
		ros::NodeHandle nh;

		for(int i=0; i<VehNum; i++){

					CurrentNode[i] = nh.subscribe<geometry_msgs::TwistStamped>(ParentNS[i]+"/currentsHat",2,boost::bind(&CurrAdapControl::onEstimate, this, _1, i));


		//			ROS_INFO("SubscriberNS = %s\n", (ParentNS[i]+"/stateHat").c_str());
				}

	}

	void onEstimate(const geometry_msgs::TwistStamped::ConstPtr& curr, int i) {

		float currentX, currentY;

		currentX = curr->twist.linear.x;
		currentY = curr->twist.linear.y;

	}


	void FormationUpdate() {



	}

private:

	ros::NodeHandle *CurrentNode;
	ros::Subscriber *StateNode;

	geometry_msgs::TwistStamped *CurrentState;
	auv_msgs::NavSts *VehState;

	int VehNum;

	std::vector<std::string> ParentNS;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "CurrentAdaptiveControl");

	CurrAdapControl currpctrl;

	ros::spin();

	return 0;
}
