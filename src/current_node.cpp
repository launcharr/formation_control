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

struct cartesian{
	double x;
	double y;
};



class CurrentNode {

	CurrentNode(){
		ros::NodeHandle nh;

		nh.param("VehNum", VehNum, 0);

		if(VehNum > 0) {
			CurrentState = new geometry_msgs::TwistStamped[VehNum];
			VehState = new auv_msgs::NavSts[VehNum];

			StateNode = new ros::Subscriber[VehNum];
			CurrentNode = new ros::Publisher[VehNum];

			init();
		}
	}

	~CurrentNode() {
		delete [] VehState;
		delete [] CurrentState;
		delete [] VehState;
		delete [] StateNode;
	}

public:

	void init() {
		ros::NodeHandle nh;

		nh.getParam("ParentNS", ParentNS);

		for(int i=0; i<VehNum; i++){

			StateNode[i] = nh.subscribe<auv_msgs::NavSts>(ParentNS[i]+"/stateHat",2,boost::bind(&CurrentNode::onEstimate, this, _1, i));

			CurrentNode[i] = nh.advertise<geometry_msgs::TwistStamped>(ParentNS[i]+"/currents",1);
		}

	}

	void onEstimate(const auv_msgs::NavSts::ConstPtr& state, const int& i) {

		ros::NodeHandle nh;
		double angle;
		std::vector<double> currFnc, VehStateCurr, VehStatei, mul;

		VehState[i] = *state;

		for(int j = 0; j < VehNum; j++){

			currFnc = CurrentFnc(VehState[j]);
			angle = atan2(currFnc[1], currFnc[0]);

			VehStateCurr[0] = VehState[j].position.east;
			VehStateCurr[1] = VehState[j].position.north;

			for(int k = 0; k < VehNum; k++) {
				if(k != j){
					VehStatei[0] = VehState[k].position.east;
					VehStatei[1] = VehState[k].position.north;

					mul = insideConeFnc(VehStateCurr, VehStatei,angle);

					currFnc[0] =* mul[0];
					currFnc[1] =* mul[1];
				}
			}
			CurrentState[j].twist.linear.x = currFnc[0];
			CurrentState[j].twist.linear.y = currFnc[1];
		}

		for(int j = 0; j < VehNum; j++){
			CurrentNode[j].publish(CurrentState[j]);
		}
	}


	std::vector<double> CurrentFnc(auv_msgs::NavSts state){

		std::vector<double> curr;

		curr[0] = -0.2;
		curr[1] = -0.2;

		return curr;

	}

	std::vector<double> insideConeFnc(const std::vector<double> Vc, const std::vector<double> Vi, const double angle){

		std::vector<double> Vir, currMul;

		Vir = rotateVectAP(Vi, Vc,-angle);

		// if is inside quadratic function
		if( Vir[0] - Vc[0] < -4*(Vir[1] - Vc[1]) + 4 && Vir[0] > 0){

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


		std::vector<double> tmp, res;

		// substract origin
		tmp[0] = p1[0] - p[0];
		tmp[1] = p1[1] - p[1];

		res[0] = tmp[0]*cos(angle) - tmp[1]*sin(angle);
		res[1] = tmp[0]*sin(angle) + tmp[1]*cos(angle);

		res[0] =+ p[0];
		res[1] =+ p[1];

		return res;
	}


private:

	ros::NodeHandle *CurrentNode;
	ros::Subscriber *StateNode;

	geometry_msgs::TwistStamped *CurrentState;
	auv_msgs::NavSts *VehState;

	int VehNum;

	std::vector<std::string> ParentNS;


};

int main(int argc, char **argv) {

	ros::init(argc, argv, "current_node");

	CurrentNode curr_node;

	ros::spin();

	return 0;
}
