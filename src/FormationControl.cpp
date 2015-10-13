#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>

void init() {}

void onManRef(const std_msgs::Bool::ConstPtr& state) {}

void windup(const auv_msgs::BodyForceReq& tauAch) {}

void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
		const auv_msgs::BodyVelocityReq& track) {}

void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state) {}

void initialize_controller() {}


int main(int argc, char **argv)  {

ros::init(argc, argv, "FormationControl");

ros::spin();

return 0;
}
