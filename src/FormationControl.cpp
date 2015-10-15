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

		ROS_INFO("\n%d. vozilo: \n", i+1);
		ROS_INFO("Kutna brzina [%f]\n", state->orientation_rate.yaw);
		ROS_INFO("Kut: [%f]\n", state->orientation.yaw);
	}

	void init() {
		ros::NodeHandle nh;

		std::string nspace[3] = {"vehicle1", "vehicle2", "vehicle3"};

		for(int i=0; i<3; i++){
			state_node[i] = nh.subscribe<auv_msgs::NavSts>(nspace[i]+"/stateHat",1,boost::bind(&FormControl::step1, this, _1, i));
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

		nh.param<bool>("FCEnable",FCEnable, false);
		nh.getParam("VehNum", VehNum);
		nh.getParam("DGMat", DGMat);
		nh.getParam("GMat", GMat);
		nh.getParam("vehNS", vehNS);

	}

private:
	ros::Subscriber state_node[];
	auv_msgs::NavSts veh_state;
	bool FCEnable;
	int VehNum;
	int DGMat[]; // Direct graph matrix
	float GMat[]; // Gain matrix
	std::string vehNS[];

};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl form;

	form.init();

	ros::spin();

return 0;
}
