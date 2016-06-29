#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <formation_control/Formation.h>

#define scaleX 1
#define scaleY 1


class FormControl {

public:

	FormControl(){
		ros::NodeHandle nh;

		ControlEnable = nh.subscribe<std_msgs::Bool>("/FCEnable", 2, &FormControl::EnableController, this);
		ReinitControl = nh.subscribe<std_msgs::Bool>("/FCReinit", 2,&FormControl::onReinitializeController, this);

		FCEnable = false;
		nh.param("VehNum", VehNum, 0);

		if(VehNum > 0) {
			FCGotState = new bool[VehNum];
			VehState = new auv_msgs::NavSts[VehNum];
			StateNode = new ros::Subscriber[VehNum];
			StateTime = new ros::Time[VehNum];
			init();
		}
	}

	~FormControl() {
		delete [] FCGotState;
		delete [] VehState;
		delete [] StateNode;
		delete [] StateTime;
	}

	void init() {
		ros::NodeHandle nh, nhSub, nhPub;
		char surf[] = "/surface";
		ROS_INFO("\n\n\n\nControler init...\n\n\n\n");

		nh.getParam("ParentNS", ParentNS);
		nh.getParam("CurrentVeh", CurrentVeh);

		/* subscribe to all vehicle states */
		for(int i=0; i<VehNum; i++){
//				ROS_INFO("i = %d\n", i);

			if(ParentNS[i] != "/surface") {
				StateNode[i] = nhSub.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",2,boost::bind(&FormControl::onEstimate, this, _1, i));
				ROS_INFO("\n\n\nVEHICLE\n\n\n");
			}
			else {
				StateNode[i] = nhSub.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/position",2,boost::bind(&FormControl::onEstimate, this, _1, i));
				ROS_INFO("\n\n\nSURFACE\n\n\n");
			}
//			ROS_INFO("SubscriberNS = %s\n", (ParentNS[i]+"/stateHat").c_str());
		}

		/* pusblish velocity request */
		VelConNode = nhPub.advertise<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+"/nuRef", 1);

		/* publish desired position to DP */
		VehPosRef = nhPub.advertise<auv_msgs::NavSts>(ParentNS[CurrentVeh]+"/PosRef", 1);

		/* subscribe to DP velocity */
		VelRef = nh.subscribe<auv_msgs::BodyVelocityReq>(ParentNS[CurrentVeh]+"/FormVel", 1, &FormControl::onControllerRef, this);

		/* subscribe to formation position reference */
		FormPosRef = nh.subscribe<auv_msgs::NavSts>("/FormPosRef", 1, &FormControl::onPosRef, this);

		/* configure velocity controller service */
		ConfVelCon = nhPub.serviceClient<navcon_msgs::ConfigureVelocityController>(ParentNS[CurrentVeh]+"/ConfigureVelocityController");

		/* enable dynamic positioning service */
		EnableDP = nhSub.serviceClient<navcon_msgs::EnableControl>(ParentNS[CurrentVeh]+"/FormPos_Enable");

		/* formation change topic */
		FormChange = nh.subscribe<formation_control::Formation>("/FormChange", 1, &FormControl::onFormationChange, this);


//		ROS_INFO("PublisherNS = %s\n", (ParentNS[CurrentVeh]+"/nuRef").c_str());

		initialize_controller();
		ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");

	}

	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, const int& i) {

		ros::NodeHandle nh;

		/* extract necessary info */
		VehState[i] = *state;
		FCGotState[i] = true;
		StateTime[i].sec = ros::Time::now().sec;

		/* if state of the vehicle didn't came in last 2 seconds, disable controller for his measurements */
		for(int j=0; j<VehNum; j++) {
			if(ros::Time::now().sec - StateTime[j].sec > 2.0) {
				FCGotState[j] = false;
			}
		}
        
		/* if controller is started and you have all necessary info, start the controller */
		if(FCGotState[CurrentVeh] && FCEnable) {
			ControlLaw();
			//ROS_INFO("\nEstimate");
		}
	}

	void ControlLaw() {

		double XCurr, YCurr, Xi, Yi, VxCurr, VyCurr, Vxi, Vyi;
		double G, FX, FY, YawCurr, YawRateCurr, yaw, speedX, speedY;
		double normalX, normalY, rij, frcX, frcY;
		int DG;
		ros::NodeHandle nh;
		YCurr = VehState[CurrentVeh].position.east;
		XCurr = VehState[CurrentVeh].position.north;
		VxCurr = VehState[CurrentVeh].body_velocity.x;
		VyCurr = VehState[CurrentVeh].body_velocity.y;
		YawCurr = VehState[CurrentVeh].orientation.yaw;
		YawRateCurr = VehState[CurrentVeh].orientation_rate.yaw;

		RplFrcX = 0.0; // x
		RplFrcY = 0.0; // y

		VelConReq.twist.linear.x = 0;
		VelConReq.twist.linear.y = 0;

		for(int i=0; i<VehNum; i++){
			if(i!=CurrentVeh && FCGotState[i]){
				Yi = VehState[i].position.east;
				Xi = VehState[i].position.north;
				DG = DGMat[CurrentVeh*VehNum + i];
				G = GMat[CurrentVeh*VehNum + i];
				FX = scaleX*FormX[CurrentVeh*VehNum + i];
				FY = scaleY*FormY[CurrentVeh*VehNum + i];
				Vxi = VehState[i].body_velocity.x;
				Vyi = VehState[i].body_velocity.y;

				//ROS_INFO("Pogreska po X %d = %f\n", i,XCurr - Xi + FX);
				//ROS_INFO("Pogreska po Y %d = %f\n", i,YCurr - Yi + FY);

				/* consensus control*/
				VelConReq.twist.linear.x = VelConReq.twist.linear.x - DG*G*(XCurr - Xi + FX);
				VelConReq.twist.linear.y = VelConReq.twist.linear.y - DG*G*(YCurr - Yi + FY);

				/* repelling force */
				rij = sqrt(pow(XCurr - Xi,2) + pow(YCurr - Yi, 2));

				if(rij < rf && UseRepel) {
					normalX = (XCurr - Xi)/sqrt(1 + ni*pow(rij,2));
					normalY = (YCurr - Yi)/sqrt(1 + ni*pow(rij,2));
					RplFrcX = RplFrcX + (kf/pow(rij,2) + kd)*normalX;
					RplFrcY = RplFrcY + (kf/pow(rij,2) + kd)*normalY;
					// add aditional force for vehicle passing
					/* frcX = RplFrcX;
					frcY = RplFrcY;
					rotateVector(frcX,frcY, 1.57);
					RplFrcX += 0.5*frcX;
					RplFrcY += 0.5*frcY; */

				}
//				ROS_INFO("FORCE X = %f Y = %f\n",RplFrcX,RplFrcY);

			}
		}
        //ROS_INFO("Vel cons = %f, %f\n", VelConReq.twist.linear.x, VelConReq.twist.linear.y);
		/* saturate before adding force*/
		saturateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, MaxSpeed);

		// add repelling force and saturate
		VelConReq.twist.linear.x += RplFrcX;
		VelConReq.twist.linear.y += RplFrcY;
//		saturateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, MaxSpeedX);

		/* rotate from NED to robot base coordinate system */
		rotateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, -YawCurr - Ts*YawRateCurr);


		/* add dynamic position */
		if(!UseExtCon && DPStart) {
			// internal DP controller
			FormVelX = -kdp*(XCurr - FormPosX);
			FormVelY = -kdp*(YCurr - FormPosY);
			//ROS_INFO("FormPos = %f, %f\n", FormPosX, FormPosY);
			//ROS_INFO("PosCurr = %f, %f\n", XCurr, YCurr);
			rotateVector(FormVelX, FormVelY, - YawCurr - Ts*YawRateCurr);
		}

		saturateVector(FormVelX, FormVelY, MaxSpeed);
		VelConReq.twist.linear.x += FormVelX;
		VelConReq.twist.linear.y += FormVelY;

		/* saturate requested speeds */
		saturateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, MaxSpeed);

		//ROS_INFO("Vel = %f, %f\n", VelConReq.twist.linear.x, VelConReq.twist.linear.y);

		/* disable axis*/
		VelConReq.disable_axis.z = true;
		VelConReq.disable_axis.pitch = true;
		VelConReq.disable_axis.roll = true;
		VelConReq.disable_axis.yaw = true;
		VelConReq.header.stamp = ros::Time::now();
		/* publish requested velocity */
		VelConNode.publish(VelConReq);
		//ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");

	}

	void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
			const auv_msgs::BodyVelocityReq& track) {}

	void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

	void onControllerRef(const auv_msgs::BodyVelocityReq::ConstPtr& ref) {

//		ROS_INFO(" DP velocity = %f, %f\n",ref->twist.linear.x,ref->twist.linear.x);
		if(UseExtCon) {
			/* external DP controller*/
			FormVelX = ref->twist.linear.x;
			FormVelY = ref->twist.linear.y;
		}
	}

	void onPosRef(const auv_msgs::NavSts::ConstPtr& ref) {

		/* formation center position reference*/
		PosRef = *ref;

		DPStart = true;

		if(UseExtCon) {

			ROS_INFO("FormPosExt = %f, %f\n", PosRef.position.north, FormPosY = PosRef.position.east);

			auv_msgs::NavSts ControllerRef = PosRef;

//			ROS_INFO(" Position = %f, %f\n",ref->position.north,ref->position.east);

			addFormCentre(ControllerRef.position.north, ControllerRef.position.east);
//			ROS_INFO(" Position = %f, %f\n",ControllerRef.position.north,ControllerRef.position.east);

			ControllerRef.header.stamp = ros::Time::now();
			VehPosRef.publish(ControllerRef);

		}
		else {
			FormPosX = PosRef.position.north;
			FormPosY = PosRef.position.east;
			ROS_INFO("FormPos2 = %f, %f\n", PosRef.position.north, FormPosY = PosRef.position.east);
			addFormCentre(FormPosX, FormPosY);
			ROS_INFO("FormPos2 = %f, %f\n", FormPosX, FormPosY);
		}


	}

	inline void rotateVector(double& vectX, double& vectY, const double& angle) {

		double tempX = vectX, tempY = vectY;

		vectX = tempX*cos(angle) - tempY*sin(angle);
		vectY = tempX*sin(angle) + tempY*cos(angle);

	}

	void rotateFormation(std::vector<double>& formx, std::vector<double>& formy, const int num, const double angle) {

		for(int i = 0; i<num*num; i++) {
			rotateVector(formx[i], formy[i], angle);
		}
	}

	void onFormationChange(const formation_control::Formation::ConstPtr& form) {

		if(form->enableParam[0]) {
			/*change formation*/
			for(int i=0; i<VehNum*VehNum; i++) {
				FormX[i] = form->FormX[i];
				FormY[i] = form->FormY[i];
				ROS_INFO("FormX[%d] = %f, FormY[i] = %f",i,FormX[i],i,FormY[i]);
			}
		}

		if(form->enableParam[1]) {
			/*only rotate formation*/
			rotateFormation(FormX, FormY, VehNum, 3.1415/180*form->FormYaw);
		}

		if( (form->enableParam[0] || form->enableParam[1]) && DPStart) {
			/*Change position reference*/
			if(UseExtCon) {
				auv_msgs::NavSts ControllerRef = PosRef;

				addFormCentre(ControllerRef.position.north, ControllerRef.position.east);
                ROS_INFO("V2 pos ref = %f, %f\n", ControllerRef.position.north, ControllerRef.position.east);
				VehPosRef.publish(ControllerRef);
			}
			else {
				FormPosX = PosRef.position.north;
				FormPosY = PosRef.position.east;

				addFormCentre(FormPosX, FormPosY);
			}
		}

	}

	void EnableController(const std_msgs::Bool::ConstPtr& enable) {
	
	    navcon_msgs::EnableControl en;
		FCEnable = enable->data;
        
        if (UseExtCon) {
            en.request.enable = enable->data;
			if(ros::service::waitForService(ParentNS[CurrentVeh]+"/FormPos_Enable", 10000)) {
				EnableDP.call(en);
            }
			if (!FCEnable) {
				DPStart = false;
			}
        }
	}

	inline void addFormCentre(double& refX, double& refY) {

		for(int i=0; i<VehNum; i++) {
			refX -= FormX[CurrentVeh*VehNum + i]/VehNum;
			refY -= FormY[CurrentVeh*VehNum + i]/VehNum;
		}

	}

	inline void saturate(double& num, const double& maxVal, const double& minVal) {

		if(num > maxVal)
			num = maxVal;
		else if (num < minVal)
			num = minVal;
	}

	inline void saturateVector(double& numX, double& numY, const double& dist) {

		double tempX = numX, tempY = numY;

		if(sqrt(pow(tempX,2) + pow(tempY,2)) > dist) {
			numX = tempX*dist/sqrt(pow(tempX,2) + pow(tempY,2));
			numY = tempY*dist/sqrt(pow(tempX,2) + pow(tempY,2));
		}

	}

	void initialize_controller() {
		ros::NodeHandle nh;
		navcon_msgs::ConfigureVelocityController req;
		navcon_msgs::EnableControl en;

		nh.getParam("DGMat", DGMat);
		nh.getParam("GMat", GMat);
		nh.getParam("FormX", FormX);
		nh.getParam("FormY", FormY);
		nh.param("gamma",gamma, 0.1);
		nh.param("Ts",Ts, 0.7);

		nh.param("UseExtCon",UseExtCon, false);
		if(!UseExtCon){
			nh.getParam("kdp",kdp);
		}
		else {
			en.request.enable = true;
			if(ros::service::waitForService(ParentNS[CurrentVeh]+"/FormPos_Enable", 10000))
				EnableDP.call(en);
			else {
				ROS_INFO("EXTERNAL DYNAMIC POSITIONING NOT STARTED");
				UseExtCon = false;
			}
		}
		nh.param("MaxSpeed",MaxSpeed, 1.0);

		nh.param("UseRepel",UseRepel, false);
		if(UseRepel){
			nh.getParam("kf",kf);
			nh.getParam("kd",kd);
			nh.param("ni",ni, 1.0);
			nh.param("rf",rf, 2.0);
		}

//		nh.param("nu_manual/maximum_speeds",MaxSpeed, {0.05,0.05,0.05,0,0,0.5});
		for(int i=0; i<VehNum;i++) {
			FCGotState[i] = false;
			StateTime[i] = ros::Time::now();
		}
		FCStart = false;
		DPStart = false;

		FormVelX = 0;
		FormVelY = 0;
		FormPosX = 0;
		FormPosY = 0;

//		FCTempStart = false;

		/* configure velocity controller for x, y axes */
		req.request.desired_mode[0] = 2;
		req.request.desired_mode[1] = 2;
		req.request.desired_mode[2] = -1;
		req.request.desired_mode[3] = -1;
		req.request.desired_mode[4] = -1;
		req.request.desired_mode[5] = -1;

		ros::service::waitForService(ParentNS[CurrentVeh]+"/ConfigureVelocityController", -1);
		ConfVelCon.call(req);
	}

	void onReinitializeController(const std_msgs::Bool::ConstPtr& enable) {
		ros::NodeHandle nh;

		if(enable->data) {
			delete [] FCGotState;
			delete [] VehState;
			delete [] StateNode;
			delete [] StateTime;

			FCEnable = false;
			nh.param("VehNum", VehNum, 0);

			if(VehNum > 0) {
				FCGotState = new bool[VehNum];
				VehState = new auv_msgs::NavSts[VehNum];
				StateNode = new ros::Subscriber[VehNum];
				StateTime = new ros::Time[VehNum];
				init();
			}
		}
	}

private:
	ros::Subscriber *StateNode;
	ros::Publisher VelConNode;
	ros::ServiceClient ConfVelCon;
	ros::Subscriber ControlEnable;
	ros::Subscriber ReinitControl;
	ros::Subscriber FormChange;
	ros::Subscriber FormPosRef, VelRef;
	ros::Publisher VehPosRef;
	ros::ServiceClient EnableDP;
	ros::Time *StateTime;

	auv_msgs::NavSts *VehState;
	auv_msgs::BodyVelocityReq VelConReq;
	auv_msgs::NavSts PosRef;

	double RplFrcX, RplFrcY;
	double FormVelX, FormVelY, FormPosX, FormPosY;

	bool FCEnable;
	bool FCStart;
	bool UseRepel;
	bool UseExtCon;
	bool *FCGotState;
	bool DPStart;
//	bool FCTempStart;
	int VehNum;
	int CurrentVeh;
	double gamma, Ts, kf, ni, kd, rf, kdp, MaxSpeed;
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
