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
	void step1(const auv_msgs::NavSts::ConstPtr& state, std::string str) {

		ROS_INFO("\n%s vozilo: \n", str.c_str());
		ROS_INFO("Kutna brzina [%f]\n", state->orientation_rate.yaw);
		ROS_INFO("Kut: [%f]\n", state->orientation.yaw);
	}

	void init() {
		ros::NodeHandle nh;

		std::string str[3] = {"Prvo", "Drugo", "Trece"};

		state_node[0] = nh.subscribe<auv_msgs::NavSts>("vehicle1/stateHat",1,boost::bind(&FormControl::step1, this, _1, str[0]));
		state_node[1] = nh.subscribe<auv_msgs::NavSts>("vehicle1/stateHat",1,boost::bind(&FormControl::step1, this, _1, str[1]));
		state_node[2] = nh.subscribe<auv_msgs::NavSts>("vehicle1/stateHat",1,boost::bind(&FormControl::step1, this, _1, str[2]));
		}

	void onManRef(const std_msgs::Bool::ConstPtr& state) {}

	void onEstimate() {}

	void windup(const auv_msgs::BodyForceReq& tauAch) {}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

	auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
						const auv_msgs::NavSts& state) {}


	void initialize_controller() {}

private:
	ros::Subscriber state_node[];
	auv_msgs::NavSts veh_state;

};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl form;

	form.init();

	ros::spin();

return 0;
}
