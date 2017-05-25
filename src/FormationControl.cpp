#include "../include/FormationControl.h"

using namespace labust::fcontrol;

void FormControl::init(ros::NodeHandle nh, ros::NodeHandle ph) {


	// initialize parameters
	initParams(nh, ph);

	/* publish velocity request */
	velConNode = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);

	/* subscribe to formation position reference */
	formPosRef = nh.subscribe<auv_msgs::NavSts>(mergeNS+"/FormPosRef", 2, &FormControl::onPosRef, this);

	/* formation change topic */
	formChange = nh.subscribe<formation_control::Formation>(mergeNS+"/FormChange", 2, &FormControl::onFormationChange, this);

	/* vehicle state node subscribe */
	vehObj[0].stateSub = nh.subscribe<auv_msgs::NavSts>("position",2,boost::bind(&FormControl::onState, this, _1, vehObj[0].id));
	//
	vehObj[0].nsState = vehObj[0].stateSub.getTopic().c_str();

	controlEnable = nh.subscribe<std_msgs::Bool>("FCEnable", 2, &FormControl::onEnableController, this);

	reinitControl = nh.subscribe<std_msgs::Bool>("FCReinit", 2, &FormControl::onReinitializeController, this);

	// add myself in formation
	formResizePub = nh.advertise<formation_control::FormVehObj>(mergeNS+"/FormResize",5);

	// add others in my formation
	formResizeSub = nh.subscribe<formation_control::FormVehObj>(mergeNS+"/FormResize",5, &FormControl::onFormationResize, this);

	/* publish desired position to DP */
	//vehPosRef = nh.advertise<auv_msgs::NavSts>("PosRef", 1);

	/* subscribe to DP velocity */
	//velRef = nh.subscribe<auv_msgs::BodyVelocityReq>("FormVel", 1, &FormControl::onControllerRef, this);

	/* configure velocity controller service */
	//confVelCon = nh.serviceClient<navcon_msgs::ConfigureVelocityController>("ConfigureVelocityController");

	/* enable dynamic positioning service */
	//enableDP = nh.serviceClient<navcon_msgs::EnableControl>("FormPos_Enable");

	//	ROS_INFO("PublisherNS = %s\n", (ParentNS[CurrentVeh]+"/nuRef").c_str());

	ROS_INFO("\n\n\n\nControler init finished...\n\n\n\n");

}

void FormControl::initParams(ros::NodeHandle nh, ros::NodeHandle ph) {

	navcon_msgs::ConfigureVelocityController req;
	navcon_msgs::EnableControl en;

	// load common formation topic
	ph.getParam("mergeNS",mergeNS);

	// load vehicle id
	ph.param("id", vehObj[0].id, vehObj[0].id);
	vehObj[0].measValid = false;



	ph.getParam("DGMat", DGMat);
	ph.getParam("GMat", GMat);
	ph.getParam("formX", formX);
	ph.getParam("formY", formY);
	ph.param("gamma",gamma, 0.1);
	ph.param("Ts",Ts, 0.7);

	ph.param("useExtCon",useExtCon, false);
	if(!useExtCon){
		ph.getParam("kdp",kdp);
	}
	else {
		en.request.enable = true;
		if(ros::service::waitForService("FormPos_Enable", 10000))
			enableDP.call(en);
		else {
			ROS_INFO("EXTERNAL DYNAMIC POSITIONING NOT STARTED");
			useExtCon = false;
		}
	}
	ph.param("maxSpeed",maxSpeed, 1.0);

	ph.param("useRepel",useRepel, false);
	if(useRepel){
		ph.getParam("kf",kf);
		ph.getParam("kd",kd);
		ph.param("ni",ni, 1.0);
		ph.param("rf",rf, 2.0);
	}

//		nh.param("nu_manual/maximum_speeds",maxSpeed, {0.05,0.05,0.05,0,0,0.5});

	FCStart = false;
	DPStart = false;

	formVelX = 0;
	formVelY = 0;
	formPosX = 0;
	formPosY = 0;

//		FCTempStart = false;

	/* configure velocity controller for x, y axes */
	req.request.desired_mode[0] = 2;
	req.request.desired_mode[1] = 2;
	req.request.desired_mode[2] = -1;
	req.request.desired_mode[3] = -1;
	req.request.desired_mode[4] = -1;
	req.request.desired_mode[5] = -1;

	ros::service::waitForService("ConfigureVelocityController", -1);
	confVelCon.call(req);
}

void FormControl::onEnableController(const std_msgs::Bool::ConstPtr& enable) {

	navcon_msgs::EnableControl en;
	FCEnable = enable->data;
	formation_control::FormVehObj formResizeReq;

	ROS_INFO("Controler enable");

	// populate request message
	formResizeReq.header.stamp = ros::Time::now();
	formResizeReq.id = vehObj[0].id;
	formResizeReq.nsState = vehObj[0].nsState;

	if (enable->data) {
		// if enabling controller, advertise yourself to the formation
		formResizeReq.insert = true;
		for(int i=0;i<3;i++) {
			formResizePub.publish(formResizeReq);
		}
	}
	else {
		// if disabling controller, delete yourself from the formation
		formResizeReq.insert = false;
		for(int i=0;i<3;i++) {
			formResizePub.publish(formResizeReq);
		}
	}

	if (useExtCon) {
		en.request.enable = enable->data;
		if(ros::service::waitForService("FormPos_Enable", 10000)) {
			enableDP.call(en);
		}
		if (!FCEnable) {
			DPStart = false;
		}
	}
}

void FormControl::onReinitializeController(const std_msgs::Bool::ConstPtr& enable) {

	if(enable->data) {

	}
}

void FormControl::onFormationResize(const formation_control::FormVehObj::ConstPtr& veh) {

	ros::NodeHandle nh;

	if (veh->insert) {
		/* for insertion check if vehicle is already in the formation
		 * including vehicles with the same id as yours
		 */
		for (int i=0; i<vehObj.size();i++) {
			if (vehObj[i].id == veh->id) {
				// if yes, exit
				return;
			}
		}
		// if not, add it
		VehicleObject tmpVeh(veh->nsState,
							 veh->id,
							 false,
							 nh.subscribe<auv_msgs::NavSts>(veh->nsState,2,boost::bind(&FormControl::onState, this, _1, veh->id)));
		vehObj.push_back(tmpVeh);
		ROS_INFO("Added vehicle ID: %d",veh->id);

		// if new vehicle was added and i was in formation, advertise myself for robustness
		if (FCEnable) {
			// populate request message
			formation_control::FormVehObj formResizeReq;
			formResizeReq.header.stamp = ros::Time::now();
			formResizeReq.id = vehObj[0].id;
			formResizeReq.nsState = vehObj[0].nsState;
			formResizeReq.insert = true;

			formResizePub.publish(formResizeReq);
		}
	}
	else {
		/* for removal check if vehicle is already in the formation
		 * excluding vehicle with the same id as yours
		 */
		for (int i=1; i<vehObj.size();i++) {
			if (vehObj[i].id == veh->id) {
				// if yes, remove it
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


	if(form->enableParam[0]) {
		/*change formation*/
		for(int i=0; i<vehNum*vehNum; i++) {
			formX[i] = form->FormX[i];
			formY[i] = form->FormY[i];
			ROS_INFO("FormX[%d] = %f, FormY[%d] = %f",i,formX[i],i,formY[i]);
		}
	}

	if(form->enableParam[1]) {
		/*only rotate formation*/
		rotateFormation(formX, formY, vehNum, 3.1415/180*form->FormYaw);
	}

	if( (form->enableParam[0] || form->enableParam[1]) && DPStart) {
		/*Change position reference*/
		if(useExtCon) {
			auv_msgs::NavSts controllerRef = posRef;

			addFormCentre(controllerRef.position.north, controllerRef.position.east);
			ROS_INFO("V2 pos ref = %f, %f\n", controllerRef.position.north, controllerRef.position.east);
			vehPosRef.publish(controllerRef);
		}
		else {
			formPosX = posRef.position.north;
			formPosY = posRef.position.east;

			addFormCentre(formPosX, formPosY);
		}
	}

}

void FormControl::onState(const auv_msgs::NavSts::ConstPtr& state, const int& id) {

	ros::NodeHandle nh;
	int inx;

	//ROS_INFO("State ID: %d",id);

	// find said vehicle
	for (int i=0; i<vehObj.size();i++) {
		if (vehObj[i].id == id) {
			// remember index of id in vehObj
			inx = i;
			break;
		}
	}
	// if older measurement return
	if (vehObj[inx].lastState.header.stamp > state->header.stamp)
		return;

	// populate vehObj[inx] with data
	vehObj[inx].lastState = *state;
	vehObj[inx].measValid = true;


	/* if state of the vehicle didn't came in last 2 seconds, disable controller for his measurements */
	for(int i=0; i<vehObj.size(); i++) {
		if(ros::Time::now() - vehObj[i].lastState.header.stamp > ros::Duration(2.0)) {
			vehObj[i].measValid = false;
		}
	}

	/* if controller is started and you have all necessary info, start the controller */
	if(vehObj[0].measValid && FCEnable) {
		ControlLaw();
		//ROS_INFO("\nEstimate");
	}

}

void FormControl::ControlLaw() {

	auv_msgs::BodyVelocityReq velConReq;
	double xCurr, yCurr, xi, yi;
	double G, FX, FY, yawCurr, yaw, speedX, speedY;
	double normalX, normalY, rij, frcX, frcY;
	int DG;
	ros::NodeHandle nh;
	yCurr = vehObj[0].lastState.position.east;
	xCurr = vehObj[0].lastState.position.north;
	yawCurr = vehObj[0].lastState.orientation.yaw;

	//RplFrcX = 0.0; // x
	//RplFrcY = 0.0; // y

	velConReq.twist.linear.x = 0;
	velConReq.twist.linear.y = 0;

	for(int i=1; i<vehObj.size(); i++){
		// if id is out of formation bounds print error
		if(vehObj[i].id < (int) sqrt(formX.size()) ) {
			// if measurement is valid add contribution
			if(vehObj[i].measValid){
				yi = vehObj[i].lastState.position.east;
				xi = vehObj[i].lastState.position.north;
				DG = DGMat[vehObj[0].id*vehObj.size() + vehObj[i].id];
				G = GMat[vehObj[0].id*vehObj.size() + vehObj[i].id];
				FX = scaleX*formX[vehObj[0].id*vehObj.size() + vehObj[i].id];
				FY = scaleY*formY[vehObj[0].id*vehObj.size() + vehObj[i].id];

				//ROS_INFO("Pogreska po X %d = %f\n", i,XCurr - Xi + FX);
				//ROS_INFO("Pogreska po Y %d = %f\n", i,YCurr - Yi + FY);

				/* consensus control */
				velConReq.twist.linear.x = velConReq.twist.linear.x - DG*G*(xCurr - xi + FX);
				velConReq.twist.linear.y = velConReq.twist.linear.y - DG*G*(yCurr - yi + FY);

				/* repelling force
				rij = sqrt(pow(XCurr - Xi,2) + pow(YCurr - Yi, 2));

				if(rij < rf && UseRepel) {
					normalX = (XCurr - Xi)/sqrt(1 + ni*pow(rij,2));
					normalY = (YCurr - Yi)/sqrt(1 + ni*pow(rij,2));
					RplFrcX = RplFrcX + (kf/pow(rij,2) + kd)*normalX;
					RplFrcY = RplFrcY + (kf/pow(rij,2) + kd)*normalY;

					// add aditional force for vehicle passing
					frcX = RplFrcX;
					frcY = RplFrcY;
					rotateVector(frcX,frcY, 1.57);
					RplFrcX += 0.5*frcX;
					RplFrcY += 0.5*frcY;

				}
				*/
				//ROS_INFO("FORCE X = %f Y = %f\n",RplFrcX,RplFrcY);
			}
		}
		else {
			ROS_ERROR("Vehicle with ID:%d out of bounds! Check formation matrix or change id.",vehObj[i].id);
		}
	}


	//ROS_INFO("Vel cons = %f, %f\n", VelConReq.twist.linear.x, VelConReq.twist.linear.y);
	/* saturate before adding force */
	saturateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, maxSpeed);
	if (sqrt( pow(velConReq.twist.linear.x,2) + pow(velConReq.twist.linear.y,2) < 0.1)) {
		velConReq.twist.linear.x = 0;
		velConReq.twist.linear.y = 0;
	}

	// add repelling force and saturate
	//VelConReq.twist.linear.x += RplFrcX;
	//VelConReq.twist.linear.y += RplFrcY;
//		saturateVector(VelConReq.twist.linear.x, VelConReq.twist.linear.y, maxSpeedX);

	/* rotate from NED to robot base coordinate system */
	rotateVector(velConReq.twist.linear.x, velConReq.twist.linear.y, - yawCurr);


	/* add dynamic position
	if(!UseExtCon && DPStart) {
		// internal DP controller
		FormVelX = -kdp*(XCurr - FormPosX);
		FormVelY = -kdp*(YCurr - FormPosY);
		//ROS_INFO("FormPos = %f, %f\n", FormPosX, FormPosY);
		//ROS_INFO("PosCurr = %f, %f\n", XCurr, YCurr);
		rotateVector(FormVelX, FormVelY, - YawCurr - Ts*YawRateCurr);
	}


	saturateVector(FormVelX, FormVelY, maxSpeed);
	VelConReq.twist.linear.x += FormVelX;
	VelConReq.twist.linear.y += FormVelY;
	*/

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

	//ROS_INFO("\n\n\nKraj publishanja zeljene brzine\n\n\n");
}




void FormControl::idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
		const auv_msgs::BodyVelocityReq& track) {}

void FormControl::reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state) {}

void FormControl::onControllerRef(const auv_msgs::BodyVelocityReq::ConstPtr& ref) {

//		ROS_INFO(" DP velocity = %f, %f\n",ref->twist.linear.x,ref->twist.linear.x);
	if(useExtCon) {
		/* external DP controller*/
		formVelX = ref->twist.linear.x;
		formVelY = ref->twist.linear.y;
	}
}

void FormControl::onPosRef(const auv_msgs::NavSts::ConstPtr& ref) {

	/* formation center position reference*/
	posRef = *ref;

	DPStart = true;

	if(useExtCon) {

		ROS_INFO("FormPosExt = %f, %f\n", posRef.position.north, formPosY = posRef.position.east);

		auv_msgs::NavSts controllerRef = posRef;

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
		addFormCentre(formPosX, formPosY);
		ROS_INFO("FormPos2 = %f, %f\n", formPosX, formPosY);
	}


}



int main(int argc, char **argv)  {

	ros::init(argc, argv, "FormationControl");
	FormControl fctrl;


	ros::spin();

return 0;
}
