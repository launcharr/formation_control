#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>


class CurrentNode {

public:

	CurrentNode(){
		ros::NodeHandle nh;

		ROS_INFO("PARAM IN!!!!!\n\n\n");
		nh.param("VehNum", VehNum, 0);
		ROS_INFO("PARAM OUTO %d!!!!!\n\n\n", VehNum);

		if(VehNum > 0) {
			VehState = new auv_msgs::NavSts[VehNum];

			FCGotState = new bool[VehNum];
			StateNH = new ros::Subscriber[VehNum];
			CurrentNH = new ros::Publisher[VehNum];

			init();
		}
	}

	~CurrentNode() {
		delete [] FCGotState;
		delete [] VehState;
		delete [] CurrentNH;
		delete [] StateNH;
	}


	void init() {
		ros::NodeHandle nh;

		ROS_INFO("INIT START!!!!!\n\n\n");

		if (ros::param::has("ParentNS")) {

			nh.getParam("ParentNS", ParentNS);

			for(int i=0; i<VehNum; i++){

				FCGotState[i] = false;

				ROS_INFO("\n\ni = %d!!!!!\n\n\n", i);

				StateNH[i] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",1,boost::bind(&CurrentNode::onEstimate, this, _1, i));

				CurrentNH[i] = nh.advertise<geometry_msgs::TwistStamped>(ParentNS[i]+"/currents",2);
			}
		}


		ROS_INFO("INIT END!!!!!\n\n\n");
	}

	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, const int& i) {

		VehState[i] = *state;
		FCGotState[i] = true;

		updateCurrent();
	}

	void updateCurrent(){

		ros::NodeHandle nh;
		double angle;
		std::vector<double> currFnc(2), VehStateCurr(2), VehStatei(2), mul(2);

		for(int j = 0; j < VehNum; j++){

			if(FCGotState[j]){

				currFnc = CurrentFnc(VehState[j]);
				angle = atan2(currFnc[1], currFnc[0]);

				VehStateCurr[0] = VehState[j].position.north;
				VehStateCurr[1] = VehState[j].position.east;

				for(int k = 0; k < VehNum; k++) {

					if(k != j && FCGotState[k]){

						VehStatei[0] = VehState[k].position.north;
						VehStatei[1] = VehState[k].position.east;

						mul = insideConeFnc(VehStateCurr, VehStatei,angle);

						currFnc[0] *= mul[0];
						currFnc[1] *= mul[1];
					}
				}


				CurrentState.twist.linear.x = currFnc[0];
				CurrentState.twist.linear.y = currFnc[1];
				ROS_INFO("\nC[%d] = [%f, %f]\n",j,currFnc[0], currFnc[1]);
				CurrentNH[j].publish(CurrentState);
			}
		}
	}

	std::vector<double> CurrentFnc(auv_msgs::NavSts state){

		std::vector<double> curr(2);

		curr[0] = -0.2;
		curr[1] = -0.2;

		return curr;

	}

	std::vector<double> insideConeFnc(const std::vector<double> Vc, const std::vector<double> Vi, const double angle){

		std::vector<double> Vir(2), currMul(2);

		Vir = rotateVectAP(Vi, Vc,-angle);
		ROS_INFO("\nVc = [%f, %f]\nVi = [%f, %f]\nVir = [%f, %f]\n",Vc[0], Vc[1], Vi[0], Vi[1], Vir[0], Vir[1]);
		ROS_INFO("\nRV = [%f, %f]\n",Vir[0] - Vc[0], Vir[1] - Vc[1]);

		// if is inside quadratic function
		if( Vir[0] - Vc[0] < -4*(pow((Vir[1] - Vc[1]),2) - 1) && Vir[0] - Vc[0] > 0){

			currMul[0] = 0.9;
			currMul[1] = 0.9;
		}
		else{
			currMul[0] = 1.0;
			currMul[1] = 1.0;
		}

		return currMul;

	}

	std::vector<double> rotateVectAP(const std::vector<double> p1, const std::vector<double> p, const double angle){


		std::vector<double> tmp(2), res(2);

		// substract origin
		tmp[0] = p1[0] - p[0];
		tmp[1] = p1[1] - p[1];

		res[0] = tmp[0]*cos(angle) - tmp[1]*sin(angle);
		res[1] = tmp[0]*sin(angle) + tmp[1]*cos(angle);

		res[0] += p[0];
		res[1] += p[1];

		return res;
	}


private:

	ros::Publisher *CurrentNH;
	ros::Subscriber *StateNH;

	geometry_msgs::TwistStamped CurrentState;
	auv_msgs::NavSts *VehState;
	bool *FCGotState;


	int VehNum;

	std::vector<std::string> ParentNS;


};

int main(int argc, char **argv) {

	ros::init(argc, argv, "current_node");

	ROS_INFO("\n\nSTART!!!!!\n\n\n");

	CurrentNode currStart;

	ROS_INFO("\n\nSPIN!!!!!\n\n\n");
	ros::spin();

	return 0;
}
