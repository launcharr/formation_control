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

	FormControl(){
		init();
	}

	void init() {
		ros::NodeHandle nh;

		ROS_INFO("\n\n\n\nControler init...\n\n\n\n");

		nh.param("FCEnable",FCEnable, false);
		nh.getParam("VehNum", VehNum);
		NVeh = VehNum;

		if(FCEnable && VehNum > 0) {
			ROS_INFO("Vehnum = %d\n", VehNum);
			nh.getParam("ParentNS", ParentNS);
			nh.getParam("CurrentVeh", CurrentVeh);
			ROS_INFO("CurrentVeh = %d\n", CurrentVeh);

			for(int i=0; i<VehNum; i++){
//				ROS_INFO("i = %d\n", i);
				StateNode[i+1] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",2,boost::bind(&FormControl::onEstimate, this, _1, i));

//				ROS_INFO("SubscriberNS = %s\n", (ParentNS[i]+"/stateHat").c_str());
			}
			VelConNode = nh.advertise<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+"/nuRef", 1);

//			ROS_INFO("PublisherNS = %s\n", (ParentNS[CurrentVeh]+"/nuRef").c_str());
		initialize_controller();
		ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");
		}
	}

	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, const int& i) {

//			ROS_INFO("\n\n\nStanje %d. vozila. \nTrenutno vozilo %d: \n\n\n ", i+1, CurrentVeh + 1);

			ROS_INFO("onEstimate i = %d", i);
			VehState[i].position.east = state->position.east;
			VehState[i].position.north = state->position.north;
			FCGotState[i] = true;
			FCStart = true;

//			ROS_INFO("\n Vozilo = %d\n",i);
			for(int j=0; j<NVeh; j++) {
				FCStart = FCStart && FCGotState[j];
//				ROS_INFO("FCGotState[%d] = %d\n",j, FCGotState[j]);
			}
//			ROS_INFO("\n Estimate number = %d\n",i);
			if(FCStart) {
				ROS_INFO("ControlLaw\n");
				ControlLaw();
			}
//			ROS_INFO("\n\n\nIzlaz iz onEstimate\n\n\n");

	}

	void ControlLaw() {

		double XCurr, YCurr, Xi, Yi;
		XCurr = VehState[CurrentVeh].position.east;
		YCurr = VehState[CurrentVeh].position.north;

//		ROS_INFO("\nXCurr = %f \nYCurr = %f", XCurr, YCurr);
		ROS_INFO("PrevVelX = %f\n", VelConReq.twist.linear.x);
		ROS_INFO("PrevVelY = %f\n", VelConReq.twist.linear.y);

		for(int j=0; j<NVeh; j++){
			if(j!=CurrentVeh){
				Xi = VehState[j].position.east;
				Yi = VehState[j].position.north;
				VelConReq.twist.linear.x = VelConReq.twist.linear.x - gain(j)*(XCurr - Xi + scaleForm('x',j));
				VelConReq.twist.linear.y = VelConReq.twist.linear.y - gain(j)*(YCurr - Yi + scaleForm('y',j));
//				ROS_INFO("\nVelX = %f \nVelY = %f", VelConReq.twist.linear.x, VelConReq.twist.linear.y);
			}
		}
		ROS_INFO("\nVelX = %f \nVelY = %f", VelConReq.twist.linear.x, VelConReq.twist.linear.y);
		VelConNode.publish(VelConReq);
		ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");

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

	void onManRef(const std_msgs::Bool::ConstPtr& state) {}

	void windup(const auv_msgs::BodyForceReq& tauAch) {}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}


	void formationChange() {}


	void initialize_controller() {
		ros::NodeHandle nh;

		nh.getParam("DGMat", DGMat);
		nh.getParam("GMat", GMat);
		nh.getParam("FormX", FormX);
		nh.getParam("FormY", FormY);
		for(int i=0; i<NVeh;i++)
			FCGotState[i] = false;
		FCStart = false;
	}

private:
	ros::Subscriber StateNode[];
	ros::Publisher VelConNode;
	auv_msgs::NavSts VehState[];
	auv_msgs::BodyVelocityReq VelConReq;
	bool FCEnable;
	bool FCStart;
	bool FCGotState[];
	int VehNum;
	int CurrentVeh;
	int NVeh;
	std::vector<std::string> ParentNS;
	std::vector<int> DGMat; // direct graph matrix
	std::vector<double> GMat; // gain matrix
	std::vector<double> FormX, FormY; // formation distances matrix
};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl fctrl;

	ros::spin();

return 0;
}
