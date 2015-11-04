#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <formation_control/Formation.h>

#define scaleX 1
#define scaleY 1
#define MaxSpeedX 0.9
#define MaxSpeedY 0.9

class FormControl {

public:

	FormControl(){
		ros::NodeHandle nh;

		ControlEnable = nh.subscribe<std_msgs::Bool>("/FCEnable", 2, &FormControl::EnableController, this);

//		nh.param("FCEnable",FCEnable, false);
		nh.param("VehNum", VehNum, 0);
//		nh.getParam("VehNum", VehNum);
		if(VehNum > 0) {
			FCGotState = new bool[VehNum];
			VehState = new auv_msgs::NavSts[VehNum];
			StateNode = new ros::Subscriber[VehNum];
//			init();
		}
	}

	~FormControl() {
		delete [] FCGotState;
		delete [] VehState;
		delete [] StateNode;
	}

	void init() {
		ros::NodeHandle nh;

		ROS_INFO("\n\n\n\nControler init...\n\n\n\n");

		nh.getParam("ParentNS", ParentNS);
		nh.getParam("CurrentVeh", CurrentVeh);


		// subscribe to all vehicle states
		for(int i=0; i<VehNum; i++){
//				ROS_INFO("i = %d\n", i);
			StateNode[i] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",2,boost::bind(&FormControl::onEstimate, this, _1, i));

//			ROS_INFO("SubscriberNS = %s\n", (ParentNS[i]+"/stateHat").c_str());
		}

		// pusblish velocity request
		VelConNode = nh.advertise<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+"/nuRef", 1);

		//configure velocity controller service
		ConfVelCon = nh.serviceClient<navcon_msgs::ConfigureVelocityController>(ParentNS[CurrentVeh]+"/ConfigureVelocityController");

		//formation change topic
		FormChange = nh.subscribe<formation_control::Formation>("/FormChange", 1, &FormControl::formationChange, this);


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
//			nh.getParam("/FCTempStart", FCTempStart);

			if(FCStart && FCEnable) {
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

		// "manual" control for testing
		nh.param("speedX",speedX, 0.);
		nh.param("speedY",speedY, 0.);
		nh.param("yaw",yaw,0.);
		VelConReq.twist.linear.x += speedX;
		VelConReq.twist.linear.y += speedY;
		VelConReq.twist.angular.z = yaw;

		// rotate from NED to robot base coordinate system
		rotateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, YawCurr + Ts*YawRateCurr);

		// saturate requested speeds
		saturate(VelConReq.twist.linear.x, MaxSpeedX, -MaxSpeedX);
		saturate(VelConReq.twist.linear.y, MaxSpeedY, -MaxSpeedY);


		ROS_INFO("VelX = %f\n", VelConReq.twist.linear.x);
		ROS_INFO("VelY = %f\n", VelConReq.twist.linear.y);

		// disable axis
		VelConReq.disable_axis.z = true;
		VelConReq.disable_axis.pitch = true;
		VelConReq.disable_axis.roll = true;

		VelConNode.publish(VelConReq);
//		ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");

	}

	void onManRef(const std_msgs::Bool::ConstPtr& state) {}

	void windup(const auv_msgs::BodyForceReq& tauAch) {}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

	void formationChange(const formation_control::Formation::ConstPtr& form) {

		for(int i=0; i<VehNum*VehNum; i++) {
			FormX[i] = form->FormX[i];
			FormY[i] = form->FormY[i];
		}
	}

	void EnableController(const std_msgs::Bool::ConstPtr& enable) {

		if(enable){
			FCEnable = enable;
			init();
		}
		else
			FCEnable = enable;
	}

	inline void saturate(double& num, const double& maxVal, const double& minVal) {

		if(num > maxVal)
			num = maxVal;
		else if (num < minVal)
			num = minVal;
	}

	inline void rotateVector(double& vectX, double& vectY, const double& angle) {

		double tempX = vectX, tempY = vectY;

		vectX = tempX*cos(angle) + tempY*sin(angle);
		vectY = -tempX*sin(angle) + tempY*cos(angle);

	}

	void initialize_controller() {
		ros::NodeHandle nh;
		navcon_msgs::ConfigureVelocityController req;

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
//		FCTempStart = false;

		//configure velocity controller for x, y axes
		req.request.desired_mode[0] = 2;
		req.request.desired_mode[1] = 2;
		req.request.desired_mode[2] = -1;
		req.request.desired_mode[3] = -1;
		req.request.desired_mode[4] = -1;
		req.request.desired_mode[5] = -1;
		while(!ConfVelCon.call(req))
			ROS_INFO("VELOCITY CONTROLLER NOT CONFIGURED\n");

	}

private:
	ros::Subscriber *StateNode, TempStart;
	ros::Publisher VelConNode;
	ros::ServiceClient ConfVelCon;
	ros::Subscriber ControlEnable;
	ros::Subscriber FormChange;
	auv_msgs::NavSts *VehState;
	auv_msgs::BodyVelocityReq VelConReq;

	bool FCEnable;
	bool FCStart;
	bool *FCGotState;
//	bool FCTempStart;
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
