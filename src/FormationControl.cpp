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

		ROS_INFO("\n%d. vozilo:", i+1);

		if(FCEnable) {
			ROS_INFO("\nVehNum: %d\n\n", VehNum);
			ROS_INFO("\nVehNS: ");
			}
	}

	void init() {
		ros::NodeHandle nh;

		std::string nspace[3] = {"vehicle1", "vehicle2", "vehicle3"};

		for(int i=0; i<3; i++){
			state_node[i] = nh.subscribe<auv_msgs::NavSts>(nspace[i]+"/stateHat",1,boost::bind(&FormControl::step1, this, _1, i));
			}
		FormControl::initialize_controller();
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

		nh.param("/FCEnable",FCEnable, false);
		nh.param<int>("/VehNum", VehNum, 0);
		nh.getParam("/DGMat", DGMat);
		nh.getParam("/GMat", GMat);
		nh.getParam("/VehNS", VehNS);

	}

private:
	ros::Subscriber state_node[];
	auv_msgs::NavSts veh_state;
	bool FCEnable;
	int VehNum;
	std::map<std::string, int> DGMat;
	std::map<std::string, float> GMat;
	std::map<std::string, std::string> VehNS;


};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl form;

	form.init();

	ros::spin();

return 0;
}
