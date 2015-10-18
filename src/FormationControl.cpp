#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#define scaleX 1
#define scaleY 1

class FormControl {

public:

	void init() {
		ros::NodeHandle nh;

		ROS_INFO("\n\n\n\nControler init...\n\n\n\n");

		nh.param("FCEnable",FCEnable, false);
		nh.param("VehNum", VehNum, 0);


		if(FCEnable && VehNum > 0) {
			nh.getParam("VehNS", VehNS);
			nh.getParam("ParentNS", ParentNS);
			nh.getParam("CurrentVeh", CurrentVeh);

			for(int i=0; i<VehNum; i++){
				StateNode[i] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+VehNS[i]+"/stateHat",1,boost::bind(&FormControl::onEstimate, this, _1, i));
			}
			VelConNode = nh.advertise<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+VehNS[CurrentVeh]+"/nuRef", 1);
		FormControl::initialize_controller();
		}
		ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");
	}

	void ControlLaw() {

		float XCurr, YCurr, Xi, Yi;
		XCurr = VehState[CurrentVeh].position.east;
		YCurr = VehState[CurrentVeh].position.north;

		for(int i=0; i<VehNum; i++) {
			if(i!=CurrentVeh){
				Xi = VehState[i].position.east;
				Yi = VehState[i].position.north;
				VelConReq.twist.linear.x = VelConReq.twist.linear.x - gain(i)*(XCurr - Xi + scaleForm('x',i));
				VelConReq.twist.linear.y = VelConReq.twist.linear.y - gain(i)*(YCurr - Yi + scaleForm('y',i));
			}
		}
		VelConNode.publish(VelConReq);

	}

	double gain(const int i) {
		return DGMat[CurrentVeh*VehNum + i]*GMat[CurrentVeh*VehNum + i];
	}

	double scaleForm(const char a, const int i) {
		if(a=='x' || a=='X')
			return scaleX*FormX[CurrentVeh*VehNum + i];
		else
			return scaleY*FormY[CurrentVeh*VehNum + i];
	}


	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, int i) {

		ROS_INFO("\nStanje %d. vozila: ", i+1);

		VehState[i] = *state;
		ControlLaw();
	}

	void onManRef(const std_msgs::Bool::ConstPtr& state) {}

	void windup(const auv_msgs::BodyForceReq& tauAch) {}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

	auv_msgs::BodyVelocityReq step(const auv_msgs::NavSts& ref,
						const auv_msgs::NavSts& state) {}

	void formationChange() {}


	void initialize_controller() {
		ros::NodeHandle nh;

		nh.getParam("DGMat", DGMat);
		nh.getParam("GMat", GMat);
		nh.getParam("FormX", FormX);
		nh.getParam("FormY", FormY);
		VelConReq.twist.linear.x = 0;
		VelConReq.twist.linear.y = 0;
	}

private:
	ros::Subscriber StateNode[];
	ros::Publisher VelConNode;
	auv_msgs::NavSts VehState[];
	auv_msgs::BodyVelocityReq VelConReq;
	bool FCEnable;
	int VehNum;
	int CurrentVeh;
	std::vector<std::string> ParentNS;
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
