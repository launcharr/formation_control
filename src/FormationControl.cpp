#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>

class FormControl {

public:
	void step1(const auv_msgs::NavSts::ConstPtr& state, int i) {

		ROS_INFO("\n%d. vozilo: ", i+1);
		ROS_INFO("\nVehNS: %s\n\n", VehNS[i].c_str());
	}

	void init() {
		ros::NodeHandle nh;

		ROS_INFO("Controler init...\n");

		nh.param("/FCEnable",FCEnable, false);
		nh.param<int>("/VehNum", VehNum, 0);

		if(FCEnable && VehNum > 0) {
			nh.getParam("/VehNS", VehNS);

			for(int i=0; i<VehNum; i++){
				StateNode[i] = nh.subscribe<auv_msgs::NavSts>(VehNS[i]+"/stateHat",1,boost::bind(&FormControl::step1, this, _1, i));
			}
		FormControl::initialize_controller();
		}
	}

	void onManRef(const std_msgs::Bool::ConstPtr& state) {}

	void onEstimate() {}

	void windup(const auv_msgs::BodyForceReq& tauAch) {}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

	auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
						const auv_msgs::NavSts& state) {}

	void formationChange() {}


	void initialize_controller() {
		ros::NodeHandle nh;

		nh.getParam("/DGMat", DGMat);
		nh.getParam("/GMat", GMat);
		nh.getParam("/FormX", FormX);
		nh.getParam("/FormY", FormY);
	}

private:
	ros::Subscriber StateNode[];
	auv_msgs::NavSts VehState;
	bool FCEnable;
	int VehNum;
	std::vector<int> DGMat; // direct graph matrix
	std::vector<double> GMat; // gain matrix
	std::vector<double> FormX, FormY; // formation distances matrix
	std::vector<std::string> VehNS; // vehicle namespaces


};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl fctrl;

	fctrl.init();

	ros::spin();

return 0;
}
