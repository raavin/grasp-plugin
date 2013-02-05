// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include <Python.h>
#include "RobotInterface.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

RobotInterface::RobotInterface(){
	isMulti=false;
    return;
}

bool RobotInterface::runPythonCommand(string func){

	string filename = PlanBase::instance()->pythonInterface();
	FILE* file = fopen(filename.c_str(), "r");
	if(file != NULL){
//		Py_Initialize();
        PyRun_SimpleFile(file, filename.c_str());
		fclose(file);
		if( PyRun_SimpleString(func.c_str()) ){
			cout << func << endl;
			return false;
		}
//		Py_Finalize();
	}

	return true;
}

void RobotInterface::doReadFromFile() {

		char line[1024];
		Matrix3 R0;
		R0=Matrix3::Identity();
		Vector3 P0(0,0,0);

		PlanBase* tc = PlanBase::instance();
		string calibFile = "extplugin/graspPlugin/RobotInterface/data/T" + tc->targetObject->bodyItemObject->name() + ".mat";

		FILE *ifp0=NULL;
		ifp0 = fopen(calibFile.c_str(),"rb");

		if(ifp0 != NULL){
				if(! fgets(line,256,ifp0) ){
						printf("result mat: Broken format1 \n");
				}

				while(fgets(line,256,ifp0) != NULL){
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;
				}

				if(line[0] != '4'){
						printf("result mat: Broken format1 \n");
				}

				int i=0;
				while(fgets(line,256,ifp0) != NULL){
						if(sscanf(line,"%lf%lf%lf%lf",&R0(i,0),&R0(i,1),&R0(i,2),&P0(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								return;
						}
						i++;
						if(i >= 3) break;
				}

				for(int i=0;i<3;i++){
						P0(i) /= 1000.0; //model m -> mm
				}
		}


		int error = system("cp /home/vvv/Logistics/data_cap.mat extplugin/graspPlugin/RobotInterface/data"); if(error) cout << error<< endl;

		//char line[1024];
		Matrix3 Rm;
		Vector3 Pm;
		FILE *ifp=NULL;

		ifp = fopen("extplugin/graspPlugin/RobotInterface/data/data_cap.mat","rb");

		if(ifp==NULL){
				printf("No data.mat\n");
				return;
		}

		if(! fgets(line,256,ifp) ){
			printf("result mat: Broken format1 \n");
		}

		while(fgets(line,256,ifp) != NULL){
				if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;
		}

		if(line[0] != '4'){
				printf("result mat: Broken format1 \n");
		}

		int i=0;
		while(fgets(line,256,ifp) != NULL){
				if(sscanf(line,"%lf%lf%lf%lf",&Rm(i,0),&Rm(i,1),&Rm(i,2),&Pm(i)) != 4){
						printf("result mat: Broken format2 %d \n",i);
						return;
				}
				i++;
				if(i >= 3) break;
		}

		for(int i=0;i<3;i++){
				Pm(i) /= 1000.0; //model m -> mm
		}

		tc->setObjPos(Vector3(P0+R0*Pm), Matrix3(R0*Rm));

		tc->flush();

		std::cout << "Position/orientation of object was set to:" << tc->objVisPos() << tc->objVisRot() << endl;

		if( !isMulti) return;

		int cnt=0;
		while(fgets(line,256,ifp) != NULL){
			cout << line << endl;
			Matrix3 Rm;
			Vector3 Pm;
			if(line[0] == '4'){
				int i=0;
				while(fgets(line,256,ifp) != NULL){
					cout << line << endl;
					stringstream sline;
					sline << line;
					sline >> Rm(i,0);
					sline >> Rm(i,1);
					sline >> Rm(i,2);
					sline >> Pm(i);
					i++;
					if(i >= 3) break;
				}
				for(int i=0;i<3;i++){
					Pm(i) /= 1000.0; //model m -> mm
				}
				cout << Vector3(P0+R0*Pm) << Matrix3(R0*Rm) << endl;
				//tc->object()->p = Pm;
				//tc->object()->R = Rm;
				//tc->flush();
				ItemPtr temp= tc->targetObject->bodyItemObject->duplicateAll();
				tc->targetObject->bodyItemObject->parentItem()->addChildItem( temp);
				BodyItem* btemp = (BodyItem*)(temp.get());
				btemp->body()->link(0)->p = P0+R0*Pm;
				btemp->body()->link(0)->R = R0*Rm;
				ItemTreeView::mainInstance()->checkItem(btemp,true);
//				btemp->notifyKinematicStateChange();
				cnt++;
//				if(cnt > 10) break;
			}
		}


		return;
}

void RobotInterface::doRecoginitionAndRead() {

		PlanBase* tc = PlanBase::instance();
		string recogCommand = "/home/vvv/Logistics/recog_" + tc->targetObject->bodyItemObject->name() + ".sh";
		int error = system(recogCommand.c_str());
		if(error) cout << error<< endl;
		doReadFromFile();
}

void RobotInterface::doCapture() {

		int error = system("/home/vvv/Logistics/capture.sh");
		if(error) cout << error<< endl;
}

void RobotInterface::doJntCalib() {

		if(isDualArm()) {
			try {
				controllerRtc()->manipulator()->calibrateJoint();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "calibrateJoint: NO_IMPLEMENT" << endl;
				throw ;
			}
		}
}

void RobotInterface::doSrvOn() {

		if (isDualArm()){
			try {
				controllerRtc()->manipulator()->servoOn();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOn: NO_IMPLEMENT" << endl;
				throw ;
			}
		} else if (isSingleArm()) {
			try {
				controllerRtc()->manipulator_common()->servoON();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoON: NO_IMPLEMENT" << endl;
				throw ;
			}
		}
}

void RobotInterface::doSrvOff() {

		if (isDualArm()) {
			try {
				controllerRtc()->manipulator()->servoOff();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOff: NO_IMPLEMENT" << endl;
				throw ;
			}
		} else if (isSingleArm()) {
			try {
				controllerRtc()->manipulator_common()->servoOFF();
			} catch (CORBA::NO_IMPLEMENT e) {
				cerr << "servoOFF: NO_IMPLEMENT" << endl;
				throw;
			}
		}
}

void RobotInterface::doHome() {

	if (isDualArm()){
		try {
			controllerRtc()->manipulator()->goInitial();
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "goInitial: NO_IMPLEMENT" << endl;
			throw;
		}
	} else if (isSingleArm()){
		RTC::JointPos jp;
		RTC::RETURN_ID * ret;

		try {
			DoubleSeq speed;
			CORBA_SeqUtil::push_back(speed, 10);
			ret = controllerRtc()->manipulator_motion()->setMaxSpeedJoint(speed);
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setSpeedJoint: NO_IMPLEMENT" << endl;
			throw;
		}

		YamlNode& poseNode = PlanBase::instance()->body()->info()->get("standardPose");
		YamlSequence * seq = poseNode.toSequence();
		for (int i = 0; i < numJoints(); i++) {
			YamlNode& node = seq->get(i);
			CORBA_SeqUtil::push_back(jp, node.toDouble());
		}
		try {
			ret = controllerRtc()->manipulator_motion()->movePTPJointAbs(jp);
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "movePTPJointAbs: NO_IMPLEMENT" << endl;
			throw;
		}
	}
}

void RobotInterface::doMove() {

		if(isSingleArm()){
				int error = moveSingleArm();
				if (error == EXIT_FAILURE)
					cout << "warning: moveSingleArm returns " << error << endl;
		}
		else if(isDualArm()){
			    int error = moveDualArm();
			    if (error == EXIT_FAILURE)
			    	cout << "warning: moveDualArm returns " << error << endl;
		}

}

void writeAngle(const vector<double>& angle, string f)
{
	string  File = "extplugin/graspPlugin/RobotInterface/" + f;
	ofstream fout(File.c_str());

	for(size_t i=0; i<angle.size()-1; i++)
		fout << angle[i]*180/3.14 << ", ";

	fout << angle.back()*180/3.14;
}

int RobotInterface::moveSingleArm()
{
	PlanBase* tc = PlanBase::instance();

	RETURN_ID * ret;

	int numFingJoint = PlanBase::instance()->bodyItemRobot()->body()->numJoints()-numJoints();
	vector<double> tmpAngle(numFingJoint);

	int seqSize=tc->graspMotionSeq.size();

	for(int n=0; n<seqSize; n++){
		try {
			DoubleSeq speed;
			CORBA_SeqUtil::push_back(speed, 20.0);
			ret = controllerRtc()->manipulator_motion()->setMaxSpeedJoint(speed);
			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: setMaxSpeedJoint returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setMaxSpeedJoint: NO_IMPLEMENT" << endl;
			throw;
		}
		RTC::JointPos jp;
		cout << "jp[" << n << "]";
		for (int i = 0; i < numJoints(); i++) {

			CORBA_SeqUtil::push_back(jp, tc->graspMotionSeq[n].jointSeq[i]);
			cout << jp[i] << ", ";
		}
		cout << endl;
		try {
			ret = controllerRtc()->manipulator_motion()->movePTPJointAbs(jp);
			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: movePTPJointAbs returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "movePTPJointAbs: NO_IMPLEMENT" << endl;
			throw;
		}
		usleep(1300000);

		for(int i=0; i<numFingJoint; i++)
			tmpAngle[i] = tc->graspMotionSeq[n].jointSeq[numJoints()+i];


		try {
			if (tc->graspMotionSeq[n].graspingState == tc->GRASPING) {

				writeAngle(tmpAngle, tc->bodyItemRobot()->body()->name()+"Provider/close.dat");
				ret = controllerRtc()->manipulator_motion()->closeGripper();
			} else {

				writeAngle(tmpAngle, tc->bodyItemRobot()->body()->name()+"Provider/open.dat");
				ret = controllerRtc()->manipulator_motion()->openGripper();
			}

			if (ret->id != EXIT_SUCCESS) {
				cout << "warning: open/close Gripper returns " << ret->id << ": " << ret->comment << endl;
				break;
			}
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "open/close Gripper: NO_IMPLEMENT" << endl;
			throw;
		}

	}

	try {
		//ret = controllerRtc()->manipulator_motion()->openGripper();
		ret = controllerRtc()->manipulator_motion()->resume();

		if (ret->id != EXIT_SUCCESS) {
			cout << "warning: open Gripper returns " << ret->id << ": " << ret->comment << endl;
		}
	} catch (CORBA::NO_IMPLEMENT e) {
		cerr << "openGripper: NO_IMPLEMENT" << endl;
		throw;
	}

	return ret->id;
}

int RobotInterface::moveDualArm()
{

		PlanBase* tc = PlanBase::instance();
		int error = 0;
		MotionCommands::DoubleSeq mtSeq;
		int size = tc->graspMotionSeq.size();
		MotionCommands::JointPosSeq jpSeq;
		jpSeq.length(size);
		mtSeq.length(size);

		static const int angle_size = tc->graspMotionSeq[0].jointSeq.size(); //23;

		for(int i=0; i < size; i++){
			vector<double> angles(angle_size);
			convertAngles(tc->graspMotionSeq[i].jointSeq, angles);
			try {
				checkAngles(angles);
			} catch (std::vector<double> angles) {
				cout << "checkAngles failed " << endl;
				throw angles;
			}
			MotionCommands::JointPos jp(angle_size + 1);
			jp.length(angle_size + 1);
			for (int j = 0; j < angle_size; j++)
				jp[j] = angles[j];

			jpSeq[i] = jp;
			mtSeq[i] = tc->graspMotionSeq[i].motionTime;
			cout << "motion time(" << i << "):" << mtSeq[i] << endl;

		}
		cout << "movePTPJointAbsSeq" << endl;
		try {
			MotionCommands::RETURN_ID * rid = controllerRtc()->motion()->movePTPJointAbsSeq(jpSeq, mtSeq);
			cout << "ret: " << rid->id << "; " << rid->comment << endl;
		} catch (CORBA::NO_IMPLEMENT e) {
			cerr << "setSoftLimitJoint: NO_IMPLEMENT" << endl;
			throw ;
		}
		return error;
}

void RobotInterface::convertAngles(const VectorXd & seq, vector<double> & angles) {

	const double o = 180.0/3.14159;

	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++)
		angles[i] = seq(i)*o;

	if(PlanBase::instance()->bodyItemRobot()->body()->name() == "HIRO" ){
		const double off = 10.0;
		const int base[] = {15, 19};
		const double offset[3] = {-off, 0.0, off};
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++) {
				int k = base[i] + j;
				angles[k] += offset[j];
			}
		}
	}
}

void RobotInterface::checkAngles(vector<double> & angles) {
	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++) {
		if (angles[i] > 360.0) {
			throw angles;
		}
	}
}

/*
int RobotInterface::objName2Id(string objname, string obj_data){

		std::ifstream fin_obj(obj_data.c_str());

		if(!fin_obj){
			cout << obj_data <<  " not found" << endl;
			return -1;
		}

		while(fin_obj){
			string tmp_obj;
			stringstream li2;
			string line2;
			int qtmp;


			getline(fin_obj,line2);
			li2 << line2;

			li2 >> qtmp;
			li2 >> tmp_obj;

			if(tmp_obj.find(objname,0) != string::npos)
				return qtmp;
		}
		return -1;
}
*/
