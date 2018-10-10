#include "../include/FormationControl.h"

using namespace labust::fcontrol;

void FormControl::init(ros::NodeHandle nh, ros::NodeHandle ph) {

	// initialize parameters
	if(!loadParams(nh, ph)) {
		ROS_INFO("\n\n\n\nControler not initialized...\n\n\n\n");
		return;
	}

	/* publish velocity request */
	velConNode = nh.advertise<labust_msgs::BodyVelocityReq>("nuRef", 1);

	headingNode = nh.advertise<auv_msgs::NavigationStatus>("stateRef", 1);

	/* subscribe to formation position reference */
	formPosRef = nh.subscribe<auv_msgs::NavigationStatus>(mergeNS+"/FormPosRef", 2, &FormControl::onPosRef, this);

	/* formation change topic */
	formChange = nh.subscribe<formation_control::Formation>(mergeNS+"/FormChange", 2, &FormControl::onFormationChange, this);

	/* vehicle state node subscribe */
	vehObj[0].stateSub = nh.subscribe<auv_msgs::NavigationStatus>("position",2,boost::bind(&FormControl::onState, this, _1, vehObj[0].id));
	// get current namespace
	vehObj[0].nsState = vehObj[0].stateSub.getTopic().c_str();

	/* subscribe to topic for enabling the controller*/
	controlEnable = nh.subscribe<std_msgs::Bool>(mergeNS+"/FCEnable", 2, &FormControl::onEnableController, this);

	/* subscribe to topic for re-enabling the controller*/
	reinitControl = nh.subscribe<std_msgs::Bool>(mergeNS+"/FCReinit", 2, &FormControl::onReinitializeController, this);

	// add myself in formation
	formResizePub = nh.advertise<formation_control::FormVehObj>(mergeNS+"/FormResize_out",5);

	// add others in my formation
	formResizeSub = nh.subscribe<formation_control::FormVehObj>(mergeNS+"/FormResize_in",5, &FormControl::onFormationResize, this);

	if(useExtCon) {
		/* publish desired position and feed-forward speed to DP */
		vehPosRef = nh.advertise<auv_msgs::NavigationStatus>("PosRef", 1);

		/* enable dynamic positioning service */
		enableDP = nh.serviceClient<labust_msgs::EnableControl>("FormPos_Enable");
	}
	else {
		/* configure velocity controller service */
		confVelCon = nh.serviceClient<labust_msgs::ConfigureVelocityController>("ConfigureVelocityController");
	}
		//ROS_INFO("PublisherNS = %s\n", (ParentNS[CurrentVeh]+"/nuRef").c_str());


	ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");

}

bool FormControl::loadParams(ros::NodeHandle nh, ros::NodeHandle ph) {

	bool status = true;

	// load common formation topic
	ph.getParam("mergeNS",mergeNS);
	ROS_INFO("Merge NS: %s",mergeNS.c_str());

	// load vehicle id
	ph.param("id", vehObj[0].id, vehObj[0].id);

	// check if using robust formation management
	ph.param("useRobustForm", useRobustForm, useRobustForm);

	// load formation parameters and exit if not defined
	if(!ph.getParam("VehNum", VehNum)) {
		ROS_ERROR("Formation control error: VehNum parameter not defined!");
		status = false;
	}

	// check if agent ID is outside boundaries (larger than)
	if(vehObj[0].id >= VehNum) {
		ROS_ERROR("Formation control error: My ID %d out of bounds!",vehObj[0].id);
		status = false;
	}

	// check if sizes of matrices are of correct size
	ph.getParam("DGMat", DGMat);
	if (DGMat.size() != pow(VehNum,2) ) {
		ROS_ERROR("Formation control error: DGMat matrix of wrong size!");
		status = false;
	}

	ph.getParam("GMat", GMat);
	if (GMat.size() != pow(VehNum,2) ) {
		ROS_ERROR("Formation control error: GMat matrix of wrong size!");
		status = false;
	}

	ph.getParam("formX", formX);
	if (formX.size() != pow(VehNum,2) ) {
		ROS_ERROR("Formation control error: FormX matrix of wrong size!");
		status = false;
	}

	ph.getParam("formY", formY);
	if (formY.size() != pow(VehNum,2) ) {
		ROS_ERROR("Formation control error: FormY matrix of wrong size!");
		status = false;
	}

	// check if using local DP controller
	ph.param("useExtCon",useExtCon, false);

	// load maximum speed of the vehicle
	ph.param("maxSpeed",maxSpeed, 0.5);

	// load use repel parameter
	ph.param("useRepel",useRepel, false);
	if(useRepel){
		ph.getParam("kf",kf);
		ph.getParam("kd",kd);
		ph.param("ni",ni, 1.0);
		ph.param("rf",rf, 2.0);
	}

	if(!useExtCon){
		ph.getParam("kdp",kdp);
	}

	return status;
}


void FormControl::onEnableController(const std_msgs::Bool::ConstPtr& enable) {

	FCEnable = enable->data;
	formation_control::FormVehObj formResizeReq;

	ROS_INFO("Controller enable");

	// populate request message
	formResizeReq.header.stamp = ros::Time::now();
	formResizeReq.id = vehObj[0].id;
	formResizeReq.header.frame_id = '0'+vehObj[0].id;
	formResizeReq.nsState = vehObj[0].nsState;

	vehObj[0].addedTS = formResizeReq.header.stamp;
	vehObj[0].lastTS = formResizeReq.header.stamp;

	if (enable->data) {
		// if enabling controller, advertise yourself to the formation
		formResizeReq.insert = true;
		for(int i=0;i<3;i++) {
			formResizePub.publish(formResizeReq);
		}

		// and configure velcon
		if(useExtCon){
			labust_msgs::EnableControl en;

			en.request.enable = true;
			if(ros::service::waitForService("FormPos_Enable", 10000))
				enableDP.call(en);
			else {
				ROS_INFO("EXTERNAL DYNAMIC POSITIONING NOT STARTED");
				useExtCon = false;
			}

		}
		if(!useExtCon) {
			labust_msgs::ConfigureVelocityController req;

			/* configure velocity controller for x, y axes */
			req.request.desired_mode = {2,2,-1,-1,-1,2};

			ros::service::waitForService("ConfigureVelocityController", -1);
			confVelCon.call(req);

			labust_msgs::EnableControl srv;
			srv.request.enable = true;
			ros::service::call("HDG_enable",srv);
		}
	}
	else {
		// if disabling controller, delete yourself from the formation
		formResizeReq.insert = false;
		for(int i=0;i<3;i++) {
			formResizePub.publish(formResizeReq);
		}

		// and configure velcon
		if(useExtCon){
			labust_msgs::EnableControl en;

			en.request.enable = false;
			if(ros::service::waitForService("FormPos_Enable", 10000))
				enableDP.call(en);
			else {
				ROS_INFO("EXTERNAL DYNAMIC POSITIONING NOT STARTED");
				useExtCon = false;
			}

		}
		if(!useExtCon) {
			labust_msgs::ConfigureVelocityController req;

			/* configure velocity controller for x, y axes */
			req.request.desired_mode = {1,1,-1,-1,-1,1};

			labust_msgs::EnableControl srv;
			srv.request.enable = false;
			ros::service::call("HDG_enable",srv);

			ros::service::waitForService("ConfigureVelocityController", -1);
			confVelCon.call(req);
		}
	}


	//ROS_INFO("Advertised myself. My ID: %d",vehObj[0].id);
}


void FormControl::onReinitializeController(const std_msgs::Bool::ConstPtr& enable) {

	if(enable->data) {

	}
}

void FormControl::onFormationResize(const formation_control::FormVehObj::ConstPtr& veh) {

	ros::NodeHandle nh;

	// exit if ID is out of bounds
	if(veh->id >= VehNum) {
		ROS_WARN("Formation control error: Agent ID %d out of bounds!",veh->id);
		return;
	}

	if (veh->insert) {
		/* for insertion check if vehicle is already in the formation
		 * including vehicles with the same id as yours
		 */
		for (int i=0; i<vehObj.size();i++) {
			if (vehObj[i].id == veh->id) {
				// if yes, check if it is newer timestamp
				if(i != 0 && veh->header.stamp.toSec() > vehObj[i].lastTS.toSec() && useRobustForm) {
					// if newer timestamp, update local info and relay advertisement
					vehObj[i].lastTS = veh->header.stamp;

					// populate relay message
					formation_control::FormVehObj formResizeReq(*veh);
					formResizeReq.header.frame_id = '0'+vehObj[0].id;
					//formResizePub.publish(*veh);
					formResizePub.publish(formResizeReq);

					//ROS_INFO("Only relayed vehicle ID: %d\tMy ID: %d",veh->id,vehObj[0].id);
				}
				return;
			}
		}
		// if not, add it
		VehicleObject tmpVeh(veh->nsState,
							 veh->id,
							 false,
							 nh.subscribe<auv_msgs::NavigationStatus>(veh->nsState,2,boost::bind(&FormControl::onState, this, _1, veh->id)),
							 veh->header.stamp,
							 veh->header.stamp);
		vehObj.push_back(tmpVeh);
		ROS_INFO("Added vehicle ID: %d\tNS: %s",veh->id,veh->nsState.c_str());

		// if new vehicle was added and I was in formation, advertise new addition and myself for robustness
		if (FCEnable) {
			if (useRobustForm) {
				// populate relay message
				formation_control::FormVehObj formResizeReq(*veh);
				formResizeReq.header.frame_id = '0'+vehObj[0].id;
				//formResizePub.publish(*veh);
				formResizePub.publish(formResizeReq);
				//ROS_INFO("Relayed vehicle ID: %d\tMy ID: %d",veh->id,vehObj[0].id);
			}

			// populate request message
			formation_control::FormVehObj formResizeReq;
			formResizeReq.header.stamp = ros::Time::now();
			formResizeReq.header.frame_id = '0'+vehObj[0].id;
			formResizeReq.id = vehObj[0].id;
			formResizeReq.nsState = vehObj[0].nsState;
			formResizeReq.insert = true;

			// update last local timestamp
			vehObj[0].lastTS = formResizeReq.header.stamp;
			//ROS_INFO("Readvertised myself. My ID: %d",vehObj[0].id);
			// readvertise myself
			formResizePub.publish(formResizeReq);
		}
	}
	else {
		/* for removal check if vehicle is already in the formation
		 * excluding vehicle with the same id as yours
		 */
		for (int i=1; i<vehObj.size();i++) {
			if (vehObj[i].id == veh->id) {
				// if yes, relay advertisement for new removal and remove it
				if (useRobustForm) {
					// populate request message
					formation_control::FormVehObj formResizeReq(*veh);
					formResizeReq.header.frame_id = '0'+vehObj[0].id;
					formResizePub.publish(formResizeReq);
				}
				vehObj.erase(vehObj.begin()+i);
			}
		}
	}
	ROS_INFO("Formation:");
	for (int i=0; i<vehObj.size();i++) {
		ROS_INFO("\tVeh ID: %d",vehObj[i].id);
	}

	return;
}


void FormControl::onFormationChange(const formation_control::Formation::ConstPtr& form) {


	if(form->shape.enable) {
		/*change formation*/
		if(form->shape.x.size() == pow(VehNum,2) && form->shape.y.size() == pow(VehNum,2)) {
			for(int i=0; i< formX.size(); i++) {
				formX[i] = form->shape.x[i];
				formY[i] = form->shape.y[i];
				ROS_INFO("FormX[%d] = %f, FormY[%d] = %f",i,formX[i],i,formY[i]);
			}
		}
		else {
			ROS_WARN("Formation control error: Shape matrix size not of correct dimensions!");
		}
	}

	if(form->rotation.enable) {
		/*only rotate formation*/
		rotateFormation(formX, formY, (int) sqrt( (double) formX.size() ), 3.1415/180*form->rotation.angle);
	}

	if( (form->shape.enable || form->rotation.enable) && DPStart) {
		/*Change position reference*/
		if(useExtCon) {
			auv_msgs::NavigationStatus controllerRef = posRef;

			addFormCentre(controllerRef.position.north, controllerRef.position.east);
			ROS_INFO("V2 pos ref = %f, %f\n", controllerRef.position.north, controllerRef.position.east);
			vehPosRef.publish(controllerRef);
		}
	}

}


void FormControl::onState(const auv_msgs::NavigationStatus::ConstPtr& state, const int& id) {

	ros::NodeHandle nh;
	int inx=999;

	//ROS_INFO("State ID: %d",id);

	// find said vehicle
	for (int i=0; i<vehObj.size();i++) {
		if (vehObj[i].id == id) {
			// remember index of id in vehObj
			inx = i;
			break;
		}
	}
	// if vehicle not found, exit
	if(inx==999) {
		return;
	}
	// if older measurement return
	if (vehObj[inx].lastState.header.stamp > state->header.stamp)
		return;

	// populate vehObj[inx] with data
	vehObj[inx].lastState = *state;
	vehObj[inx].stateValid = true;

	//ROS_INFO("State id: %d Pos: (%f, %f)",id, state->position.north, state->position.east);

	/* if state of the vehicle didn't came in last 2 seconds, disable controller for his measurements */
	for(int i=0; i<vehObj.size(); i++) {
		if(ros::Time::now() - vehObj[i].lastState.header.stamp > ros::Duration(2.0)) {
			vehObj[i].stateValid = false;
		}
	}

	/* if controller is started and you have all necessary info, start the controller */
	if(vehObj[0].stateValid && FCEnable) {
		ControlLaw();
		//ROS_INFO("Estimate");
	}
}

void FormControl::ControlLaw() {

	labust_msgs::BodyVelocityReq velConReq;
	double xCurr, yCurr, xi, yi;
	double G, FX, FY, yawCurr;
	double normalX, normalY, rij, frcX, frcY;
	double rplFrcX(0.0), rplFrcY(0.0);
	int DG;
	ros::NodeHandle nh;
	yCurr = vehObj[0].lastState.position.east;
	xCurr = vehObj[0].lastState.position.north;
	yawCurr = vehObj[0].lastState.orientation.z;

	velConReq.twist.linear.x = 0;
	velConReq.twist.linear.y = 0;


	for(int i=1; i<vehObj.size(); i++){
		// if measurement is valid add contribution
		//ROS_INFO("ID %d valid: %d",vehObj[i].id, vehObj[i].stateValid);

		if(vehObj[i].stateValid){
			yi = vehObj[i].lastState.position.east;
			xi = vehObj[i].lastState.position.north;
			DG = DGMat[vehObj[0].id*VehNum + vehObj[i].id];
			G = GMat[vehObj[0].id*VehNum + vehObj[i].id];
			FX = formX[vehObj[0].id*VehNum + vehObj[i].id];
			FY = formY[vehObj[0].id*VehNum + vehObj[i].id];

			//ROS_INFO("dX %d = %f\tdY %d = %f", i,xCurr - xi + FX, i,yCurr - yi + FY);

			/* consensus control */
			velConReq.twist.linear.x = velConReq.twist.linear.x - DG*G*(xCurr - xi + FX);
			velConReq.twist.linear.y = velConReq.twist.linear.y - DG*G*(yCurr - yi + FY);

			if(useRepel) {
				/* repelling force */
				rij = sqrt(pow(xCurr - xi,2) + pow(yCurr - yi, 2));

				if(rij < rf) {
					normalX = (xCurr - xi)/sqrt(1 + ni*pow(rij,2));
					normalY = (yCurr - yi)/sqrt(1 + ni*pow(rij,2));
					rplFrcX = rplFrcX + (kf/pow(rij,2) + kd)*normalX;
					rplFrcY = rplFrcY + (kf/pow(rij,2) + kd)*normalY;

					// add aditional force for vehicle passing
					frcX = rplFrcX;
					frcY = rplFrcY;
					rotateVector(frcX,frcY, 1.57);
					rplFrcX += 0.5*frcX;
					rplFrcY += 0.5*frcY;

				}
			}
			//ROS_INFO("FORCE X = %f Y = %f\n",RplFrcX,RplFrcY);
		}
	}


	//ROS_INFO("Vel cons = %f, %f\n", velConReq.twist.linear.x, velConReq.twist.linear.y);


	/* saturate before adding force */
	saturateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, maxSpeed);
	if (sqrt( pow(velConReq.twist.linear.x,2) + pow(velConReq.twist.linear.y,2)) < 0.03) {
		velConReq.twist.linear.x = 0;
		velConReq.twist.linear.y = 0;
	}

	// add repelling force and saturate
	velConReq.twist.linear.x += rplFrcX;
	velConReq.twist.linear.y += rplFrcY;
	saturateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, maxSpeed);

	//ROS_INFO("Vel cons3 = %f, %f\n", velConReq.twist.linear.x, velConReq.twist.linear.y);

	/* rotate from NED to robot base coordinate system */
	rotateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, - yawCurr);

	/* add dynamic position*/
	if(!useExtCon && DPStart) {
		double currFormCentreX = 0, currFormCentreY = 0;

		// calculate current formation centre
		for(int i=0; i < vehObj.size(); i++) {
			currFormCentreX = currFormCentreX + vehObj[i].lastState.position.north;
			currFormCentreY = currFormCentreY + vehObj[i].lastState.position.east;
		}
		currFormCentreX = currFormCentreX / vehObj.size();
		currFormCentreY = currFormCentreY / vehObj.size();

		if(sqrt(pow(currFormCentreX - formPosX,2)+pow(currFormCentreY - formPosY,2)) > 0.5) {
			// internal DP controller that centers current formation
			formVelX = -kdp*(currFormCentreX - formPosX);
			formVelY = -kdp*(currFormCentreY - formPosY);
			//ROS_INFO("formPos = %f, %f\n", formPosX, formPosY);
			//ROS_INFO("PosCurr = %f, %f\n", xCurr, yCurr);
			rotateVector(formVelX, formVelY, - yawCurr);
		}
		else {
			formVelX = 0;
			formVelY = 0;
		}
	}

	saturateVector(formVelX, formVelY, maxSpeed);
	velConReq.twist.linear.x += formVelX;
	velConReq.twist.linear.y += formVelY;


	/* saturate requested speeds */
	saturateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, maxSpeed);

	//ROS_INFO("Vel = %f, %f\n", VelConReq.twist.linear.x, VelConReq.twist.linear.y);
	/* disable axis */
	velConReq.disable_axis.z = true;
	velConReq.disable_axis.pitch = true;
	velConReq.disable_axis.roll = true;
	velConReq.disable_axis.yaw = true;
	velConReq.header.stamp = ros::Time::now();

	/* publish requested velocity */
	velConNode.publish(velConReq);

	/* publish for heading controler */
	auv_msgs::NavigationStatus headingReq;
	headingReq.header.stamp = ros::Time::now();
	headingReq.orientation.z = 0;
	headingNode.publish(headingReq);

	//ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");
}



void FormControl::idle(const auv_msgs::NavigationStatus& ref, const auv_msgs::NavigationStatus& state,
		const labust_msgs::BodyVelocityReq& track) {}

void FormControl::reset(const auv_msgs::NavigationStatus& ref, const auv_msgs::NavigationStatus& state) {}

void FormControl::onPosRef(const auv_msgs::NavigationStatus::ConstPtr& ref) {

	/* formation center position reference*/
	posRef = *ref;

	DPStart = true;

	if(useExtCon) {

		ROS_INFO("FormPosExt = %f, %f\n", posRef.position.north, formPosY = posRef.position.east);

		auv_msgs::NavigationStatus controllerRef = posRef;

//			ROS_INFO(" Position = %f, %f\n",ref->position.north,ref->position.east);

		addFormCentre(controllerRef.position.north, controllerRef.position.east);
//			ROS_INFO(" Position = %f, %f\n",ControllerRef.position.north,ControllerRef.position.east);

		controllerRef.header.stamp = ros::Time::now();
		vehPosRef.publish(controllerRef);

	}
	else {
		formPosX = posRef.position.north;
		formPosY = posRef.position.east;
		ROS_INFO("FormPos2 = %f, %f\n", posRef.position.north, formPosY);
		//addFormCentre(formPosX, formPosY);
		ROS_INFO("FormPos2 = %f, %f\n", formPosX, formPosY);
	}


}

int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl fctrl;


	ros::spin();

return 0;
}
