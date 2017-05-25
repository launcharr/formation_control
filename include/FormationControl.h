/*
 * FormationControl.h
 *
 *  Created on: May 23, 2017
 *      Author: loncar
 */

#ifndef FORMATION_CONTROL_INCLUDE_LABUST_FCONTROL_FORMATIONCONTROL_H_
#define FORMATION_CONTROL_INCLUDE_LABUST_FCONTROL_FORMATIONCONTROL_H_

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
#include <formation_control/FormVehObj.h>

#define scaleX 1
#define scaleY 1

namespace labust
{
	namespace fcontrol
	{
		class FormControl {

		public:

			FormControl():FCEnable(false),vehObj(1)
			{
				ros::NodeHandle nh, ph("~");

				// initialize controller
				init(nh, ph);
			}

			~FormControl() {

			}

			// vehicle class
			class VehicleObject {

			public:

				VehicleObject():nsState(),id(),stateSub(),measValid(),lastState(){};
				VehicleObject(std::string nsState,
							  uint8_t id,
							  bool measValid,
							  ros::Subscriber stateSub)
							  :nsState(nsState),
							   id(id),
							   stateSub(stateSub),
							   measValid(measValid),
							   lastState(){};

				~VehicleObject() {
				}
				// namespace of the robot
				std::string nsState;
				// id in the swarm
				int id;
				// is measurement valid (acquired in the last 2 seconds)
				bool measValid;
				// state subscriber
				ros::Subscriber stateSub;
				// state of the vehicle
				auv_msgs::NavSts lastState;
			};

			void initParams(ros::NodeHandle nh, ros::NodeHandle ph);

			void init(ros::NodeHandle nh, ros::NodeHandle ph);

			void onFormationResize(const formation_control::FormVehObj::ConstPtr& veh);

			void onState(const auv_msgs::NavSts::ConstPtr& state, const int& i);

			void ControlLaw();

			void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
					const auv_msgs::BodyVelocityReq& track);

			void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state);

			void onControllerRef(const auv_msgs::BodyVelocityReq::ConstPtr& ref);

			void onPosRef(const auv_msgs::NavSts::ConstPtr& ref);

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


			void onFormationChange(const formation_control::Formation::ConstPtr& form);

			void onEnableController(const std_msgs::Bool::ConstPtr& enable);

			void onReinitializeController(const std_msgs::Bool::ConstPtr& enable);

			inline void addFormCentre(double& refX, double& refY) {

				for(int i=0; i<vehNum; i++) {
					refX -= formX[currentVeh*vehNum + i]/vehNum;
					refY -= formY[currentVeh*vehNum + i]/vehNum;
				}

			}

			inline void saturate(double& num, const double& maxVal, const double& minVal) {

				if(num > maxVal)
					num = maxVal;
				else if (num < minVal)
					num = minVal;
			}

			void saturateVector(double& numX, double& numY, const double& dist) {

				double tempX = numX, tempY = numY;

				if(sqrt(pow(tempX,2) + pow(tempY,2)) > dist) {
					numX = tempX*dist/sqrt(pow(tempX,2) + pow(tempY,2));
					numY = tempY*dist/sqrt(pow(tempX,2) + pow(tempY,2));
				}

			}


		private:

			// vector of vehicle objects
			std::vector<VehicleObject> vehObj;

			ros::Publisher velConNode;
			ros::Subscriber controlEnable;
			ros::Subscriber reinitControl;
			ros::Subscriber formChange;
			ros::Subscriber formPosRef;
			ros::Subscriber formResizeSub;
			ros::Publisher formResizePub;

			std::string mergeNS;

			auv_msgs::BodyVelocityReq velConReq;
			auv_msgs::NavSts posRef;

			ros::Subscriber velRef;
			ros::Publisher vehPosRef;
			ros::ServiceClient enableDP;
			ros::ServiceClient confVelCon;

			double rplFrcX, rplFrcY;
			double formVelX, formVelY, formPosX, formPosY;

			bool FCEnable;
			bool FCStart;
			bool useRepel;
			bool useExtCon;
			bool DPStart;

			int vehNum;
			int currentVeh;
			double gamma, Ts, kf, ni, kd, rf, kdp, maxSpeed;
			std::vector<int> DGMat; // direct graph matrix
			std::vector<double> GMat; // gain matrix
			std::vector<double> formX, formY; // formation distances matrix

		};
	}
}


#endif /* FORMATION_CONTROL_INCLUDE_LABUST_CONTROL_FORMATIONCONTROL_H_ */
