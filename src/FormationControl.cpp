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
#define scaleX 1
#define scaleY 1
#define MaxSpeedX 0.9
#define MaxSpeedY 0.9

class FormControl {

public:

	FormControl(){
		ros::NodeHandle nh;

		nh.param("FCEnable",FCEnable, false);
		nh.param("VehNum", VehNum, 0);
//		nh.getParam("VehNum", VehNum);
		if(FCEnable && VehNum > 0) {
			FCGotState = new bool[VehNum];
			VehState = new auv_msgs::NavSts[VehNum];
			StateNode = new ros::Subscriber[VehNum];
			init();
		}
	}

	~FormControl() {
		delete [] FCGotState;
		delete [] VehState;
		delete [] StateNode;
	}

	void init() {
		ros::NodeHandle nh;
//		ros::ServiceClient client;
//		navcon_msgs::ConfigureVelocityController req;

		ROS_INFO("\n\n\n\nControler init...\n\n\n\n");

		nh.getParam("ParentNS", ParentNS);
		nh.getParam("CurrentVeh", CurrentVeh);

//		client = nh.serviceClient<navcon_msgs::ConfigureVelocityControllerRequest>(ParentNS[CurrentVeh]+"/ConfigureVelocityController");

		ROS_INFO("CurrentVeh = %d\n", CurrentVeh);
		ROS_INFO("VehNum = %d\n", VehNum);

		for(int i=0; i<VehNum; i++){
//				ROS_INFO("i = %d\n", i);
			StateNode[i] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",2,boost::bind(&FormControl::onEstimate, this, _1, i));

//			ROS_INFO("SubscriberNS = %s\n", (ParentNS[i]+"/stateHat").c_str());
		}
		VelConNode = nh.advertise<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+"/nuRef", 1);

//		ROS_INFO("PublisherNS = %s\n", (ParentNS[CurrentVeh]+"/nuRef").c_str());
		initialize_controller();
		ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");

	}

	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, const int& i) {

		ros::NodeHandle nh;
//			ROS_INFO("\n\n\nStanje %d. vozila. \nTrenutno vozilo %d: \n\n\n ", i+1, CurrentVeh + 1);

			ROS_INFO("onEstimate i = %d", i);
			VehState[i].position.east = state->position.east;
			VehState[i].position.north = state->position.north;
			VehState[i].body_velocity.x = state->body_velocity.x;
			VehState[i].body_velocity.y = state->body_velocity.y;
			VehState[i].orientation.yaw = state->orientation.yaw;
			VehState[i].orientation_rate.yaw = state->orientation_rate.yaw;
			FCGotState[i] = true;
			FCStart = true;

//			ROS_INFO("NVeh = %d\n",NVeh);
			for(int j=0; j<VehNum; j++) {
				FCStart = FCStart && FCGotState[j];
//				ROS_INFO("FCGotState[%d] = %d\n",j, FCGotState[j]);
			}
			nh.getParam("/FCTempStart", FCTempStart);

			if(FCStart && FCTempStart) {
				ROS_INFO("ControlLaw\n");
				ControlLaw();
			}
//			ROS_INFO("\n\n\nIzlaz iz onEstimate\n\n\n");
	}

	void ControlLaw() {

		double XCurr, YCurr, Xi, Yi, VxCurr, VyCurr, Vxi, Vyi, G, FX, FY, YawCurr, YawRateCurr, yaw, speedX, speedY;
		int DG;
		ros::NodeHandle nh;
		YCurr = VehState[CurrentVeh].position.east;
		XCurr = VehState[CurrentVeh].position.north;
		VxCurr = VehState[CurrentVeh].body_velocity.x;
		VyCurr = VehState[CurrentVeh].body_velocity.y;
		YawCurr = VehState[CurrentVeh].orientation.yaw;
		YawRateCurr = VehState[CurrentVeh].orientation_rate.yaw;

//		ROS_INFO("XCurr = %f \tYCurr = %f", XCurr, YCurr);
//		ROS_INFO("Stvarna brzina u X = %f\n",VxCurr);
//		ROS_INFO("Stvarna brzina u Y = %f\n",VyCurr);

		VelConReq.twist.linear.x = 0;
		VelConReq.twist.linear.y = 0;

		for(int i=0; i<VehNum; i++){
			if(i!=CurrentVeh){
				Yi = VehState[i].position.east;
				Xi = VehState[i].position.north;
				DG = DGMat[CurrentVeh*VehNum + i];
				G = GMat[CurrentVeh*VehNum + i];
				FX = scaleX*FormX[CurrentVeh*VehNum + i];
				FY = scaleY*FormY[CurrentVeh*VehNum + i];
				Vxi = VehState[i].body_velocity.x;
				Vyi = VehState[i].body_velocity.y;

				ROS_INFO("Udaljenost od %d X = %f\n", i,XCurr - Xi);
				ROS_INFO("Udaljenost od %d Y = %f\n", i,YCurr - Yi);

				VelConReq.twist.linear.x = VelConReq.twist.linear.x - DG*G*(XCurr - Xi + FX + gamma*(VxCurr - Vxi));
				VelConReq.twist.linear.y = VelConReq.twist.linear.y - DG*G*(YCurr - Yi + FY + gamma*(VxCurr - Vxi));
			}
		}

		nh.param("speedX",speedX, 0.);
		nh.param("speedY",speedY, 0.);
		VelConReq.twist.linear.x += speedX;
		VelConReq.twist.linear.y += speedY;

		double tempX = VelConReq.twist.linear.x, tempY = VelConReq.twist.linear.y;

		VelConReq.twist.linear.x = tempX*cos(YawCurr + Ts*YawRateCurr) + tempY*sin(YawCurr + Ts*YawRateCurr);
//		VelConReq.twist.linear.y = 0;
		VelConReq.twist.linear.y = -tempX*sin(YawCurr + Ts*YawRateCurr) + tempY*cos(YawCurr + Ts*YawRateCurr);

		if(VelConReq.twist.linear.x > MaxSpeedX)
			VelConReq.twist.linear.x = MaxSpeedX;
		else if (VelConReq.twist.linear.x < -MaxSpeedX)
			VelConReq.twist.linear.x = -MaxSpeedX;

		if(VelConReq.twist.linear.y > MaxSpeedY)
			VelConReq.twist.linear.y = MaxSpeedY;
		else if (VelConReq.twist.linear.y < -MaxSpeedY)
			VelConReq.twist.linear.y = -MaxSpeedY;

		nh.param("yaw",yaw,0.);
		VelConReq.twist.angular.z = yaw;

		ROS_INFO("VelX = %f\n", VelConReq.twist.linear.x);
		ROS_INFO("VelY = %f\n", VelConReq.twist.linear.y);

		VelConNode.publish(VelConReq);
//		ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");

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
		nh.param("gamma",gamma, 0.1);
		nh.param("Ts",Ts, 0.7);
//		nh.param("nu_manual/maximum_speeds",MaxSpeed, {0.05,0.05,0.05,0,0,0.5});
		for(int i=0; i<VehNum;i++)
			FCGotState[i] = false;
		FCStart = false;
		FCTempStart = false;
	}

private:
	ros::Subscriber *StateNode, TempStart;
	ros::Publisher VelConNode;
	auv_msgs::NavSts *VehState;
	auv_msgs::BodyVelocityReq VelConReq;

	bool FCEnable;
	bool FCStart;
	bool *FCGotState;
	bool FCTempStart;
	int VehNum;
	int CurrentVeh;
	double gamma, Ts;
	std::vector<std::string> ParentNS;
	std::vector<int> DGMat; // direct graph matrix
	std::vector<double> GMat; // gain matrix
	std::vector<double> FormX, FormY; // formation distances matrix
	std::vector<double> MaxSpeed;
};

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl fctrl;

	ros::spin();

return 0;
}
