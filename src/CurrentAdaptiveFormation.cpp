#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>


#include <auv_msgs/NavigationStatus.h>
#include <std_msgs/Bool.h>
#include <formation_control/Formation.h>
#include <geometry_msgs/TwistStamped.h>


class CurrAdapControl {

	public:

	CurrAdapControl() {

		ros::NodeHandle nh;

		AdaptFCEnableNH = nh.subscribe<std_msgs::Bool>("/AdaptFCEnable", 2, &CurrAdapControl::EnableController, this);

		AdaptFCEnable = false;

		nh.param("VehNum", VehNum, 0);

		ROS_INFO("VEHNUM = %d!!!!!\n\n\n",VehNum);

		if(VehNum > 0) {

			CurrentState = new geometry_msgs::TwistStamped[VehNum];
			//VehState = new auv_msgs::NavigationStatus[VehNum];

			FCGotCurr = new bool[VehNum];
			CurrentNH = new ros::Subscriber[VehNum];
			//StateNH = new ros::Subscriber[VehNum];
			init();
		}
	}

	~CurrAdapControl() {
		//delete [] VehState;
		delete [] CurrentState;
		delete [] CurrentNH;
		//delete [] StateNH;
	}



	void init() {
		ros::NodeHandle nh;

		// get parameters
		nh.getParam("ParentNS", ParentNS);
		nh.getParam("FormLineX", FormLineX);
		nh.getParam("FormLineY", FormLineY);
		nh.param("FormDist", FormDist, 1.0);
		nh.param("deltaPhi", deltaPhi, 0.0005);

		for(int j=0; j < VehNum; j++) {
			for(int k=0; k < VehNum; k++) {
				FormLineX[j*VehNum + k] *= FormDist;
				FormLineY[j*VehNum + k] *= FormDist;
			}
		}

		for(int i=0; i<VehNum; i++){

			FCGotCurr[i] = false;

			CurrentNH[i] = nh.subscribe<geometry_msgs::TwistStamped>(ParentNS[i]+"/currentsHat",2,boost::bind(&CurrAdapControl::onEstimate, this, _1, i));
			FormationNH = nh.advertise<formation_control::Formation>("/FormChange",1);

		}

	}

	void onEstimate(const geometry_msgs::TwistStamped::ConstPtr& curr, int i) {

		int k = 0;
		double CurrDirectionNew = 0.0;

		ROS_INFO("\nGot current estimation %.3f.\n", curr->twist.linear.x);

		CurrentState[i] = *curr;
		FCGotCurr[i] = true;

		if(AdaptFCEnable) {
			for(int j=0; j<VehNum; j++) {
				if(FCGotCurr[j]) {
					CurrDirectionNew += atan2(CurrentState[j].twist.linear.y, CurrentState[j].twist.linear.x);
					k++;
				}
			}
			CurrDirectionNew = CurrDirectionNew/k;

			//if(abs(CurrDirection - CurrDirectionNew) > deltaPhi){

				CurrDirection = CurrDirectionNew;

				FormationUpdate(CurrDirection);
			//}
		}
	}


	void FormationUpdate(const double angle) {

		FormReq.shape.x = FormLineX;
		FormReq.shape.y = FormLineY;
		FormReq.shape.enable = true;
		FormReq.rotation.angle = 180*angle/3.1415;
		FormReq.rotation.enable = true;

		FormationNH.publish(FormReq);

	}

	void EnableController(const std_msgs::Bool::ConstPtr& enable) {

		ROS_INFO("ENABLED!!!!!\n\n\n");
		AdaptFCEnable = enable->data;
		CurrDirection = 0.0;

	}



private:

	ros::Subscriber *CurrentNH;
	//ros::Subscriber *StateNH;
	ros::Subscriber AdaptFCEnableNH;
	ros::Publisher FormationNH;

	geometry_msgs::TwistStamped *CurrentState;
	//auv_msgs::NavigationStatus *VehState;
	formation_control::Formation FormReq;

	double deltaPhi, FormDist;
	double CurrDirection;

	int VehNum;

	bool *FCGotCurr, AdaptFCEnable;

	std::vector<std::string> ParentNS;

	std::vector<double> FormLineX, FormLineY;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "CurrentAdaptiveControl");

	CurrAdapControl currpctrl;

	ros::spin();

	return 0;
}
