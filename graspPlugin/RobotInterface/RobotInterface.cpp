// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include <Python.h>
#include "RobotInterface.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/RootItem>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/PoseSeqItem>	/* modified by qtconv.rb 0th rule*/
#include "../Grasp/GraspController.h"
#include "../GripperManipulation/RobotLocalFunctions.h"
#include "HIROControllerRtc.h"

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


		//system("./auto_scp.sh");
		int error = system("cp ~/work/cap/data_cap.mat extplugin/graspPlugin/RobotInterface/data"); if(error) cout << error<< endl;

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
		string recogCommand = "/home/vvv/work/cap/recog_" + tc->targetObject->bodyItemObject->name() + ".sh";
		int error = system(recogCommand.c_str());
		if(error) cout << error<< endl;
		doReadFromFile();
}

void RobotInterface::doCapture() {
		int error = system("/home/vvv/work/cap/capture.sh");
		if(error) cout << error<< endl;
}

void RobotInterface::doJntCalib() {

		if(PlanBase::instance()->bodyItemRobot()->body()->name()=="HIRO"){
			HIROControllerRtc::instance()->comp_->manipulator()->calibrateJoint();
		}


}
void RobotInterface::doSrvOn() {

		//string func = "doServoOn()";
		//runPythonCommand(func);

		if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10"){
				int error = system("manip start");
				error = system("manip serv_on");
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10_VVV"){
				int error = system("/usr/local/Robot/bin/pa10-q -s pa10server init 2> /dev/null");
				error = system("/usr/local/Robot/bin/pa10-q -s pa10server restart");
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="HIRO"){
				HIROControllerRtc::instance()->comp_->manipulator()->servoOn();
		}

//		string func = "print multiply_(2,3)";
//		runPythonCommand(func);

}

void RobotInterface::doSrvOff() {

		//string func = "doServoOff()";
		//runPythonCommand(func);
		if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10"){
				int error = system("manip serv_off");
				error = system("manip end");
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10_VVV"){
				int error = system("/usr/local/Robot/bin/pa10-q -s pa10server \"speed 10\""); if(error) cout << error<< endl;
				error =system("/usr/local/Robot/bin/pa10-q -s pa10server \" rotate 0.0 0.0 0.0 0.0 0.0 0.0 0.0 \" ");if(error) cout << error<< endl;
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="HIRO"){
				HIROControllerRtc::instance()->comp_->manipulator()->servoOff();
		}

}

void RobotInterface::doHome() {
		//string func = "gotoHomePosition()";
		//runPythonCommand(func);
		int error;
		if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10"){
				error = system("manip abs_jmove 0.0 0.0 0.0 90.0 0.0 90.0 0.0");
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10_VVV"){
				error = system("/usr/local/Robot/bin/pa10-q -s pa10server \"speed 10\""); if(error) cout << error<< endl;
				error = system("/usr/local/Robot/bin/pa10-q -s pa10server \" rotate 0.0 0.0 0.0 90.0 180.0 -90.0 0.0 \" "); if(error) cout << error<< endl;

		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="HIRO"){
				HIROControllerRtc::instance()->comp_->manipulator()->goInitial();
		}
}

void RobotInterface::doMove() {

		PlanBase* tc = PlanBase::instance();

		if(tc->bodyItemRobot()->body()->name()=="PA10"){
				ifstream gin("extplugin/graspPlugin/RobotInterface/data/grasp.mat");

				double t;
				gin >> t;
				gin >> t;

				while(!gin.eof()){

						int error = system("manip set_max_ang_vel 20.0");
						if (error != 0) {
							cout << "warning: manip returns " << error << endl;
						}

						string command= "manip abs_jmove";
						char chr[256];

						for(int i=0; i<tc->arm()->arm_path->numJoints(); i++){
								gin >> t;
								sprintf(chr, "%f", t);
								command = command + " " + string(chr);
						}

						cout << command << endl;

						error = system(command.c_str());
						usleep(1300000);

						gin >> t;
						gin >> t;
						gin >> t;
						if((int)t == tc->GRASPING)
								error = system("manip grasp");
						else
								error = system("manip hand_open");
				}

				int error = system("manip hand_open");
				if (error != 0) {
					cout << "warning: manip hand_open returns " << error << endl;
				}
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="PA10_VVV"){
				int error = system("/home/Robot/freeformdemo/shells/grasp_object_excade.sh");
				if(error) cout << error<< endl;
		}
		else if(PlanBase::instance()->bodyItemRobot()->body()->name()=="HIRO"){
			    int error = HIROMove();
			    if (error) {
			    	cout << "warning: Hiro returns " << error << endl;
			    }
		}

}

int RobotInterface::HIROMove()
{
	static const double offset = 10.0;

	GripperManipulation::RobotLocalFunctions rb;

	PlanBase* tc = PlanBase::instance();
	int error = 0;
	vector<VectorXd>& jointSeq = tc->jointSeq;
	vector<double> & motionTimeSeq = tc->motionTimeSeq;
	MotionCommands::DoubleSeq mtSeq;
	int size = jointSeq.size();
	MotionCommands::JointPosSeq jpSeq;
	jpSeq.length(size);
	mtSeq.length(size);
	static const int angle_size = 23;
	for (int i = 0; i < size; i++){
		vector<double> angles(angle_size);
		rb.convertAngles(jointSeq[i], angles, offset);
		try {
			rb.checkAngles(angles);
		} catch (std::vector<double> angles) {
			char jointlogfile[256];
			rb.writeJointSeq(jointlogfile, 255, jointSeq, motionTimeSeq);
			cout << "Wrong jointSeq! Check " << jointlogfile << endl;
			throw angles;
		}
		MotionCommands::JointPos jp(angle_size + 1);
		jp.length(angle_size + 1);
		for (int j = 0; j < angle_size; j++) {
			jp[j] = angles[j];
		}
		jpSeq[i] = jp;
		mtSeq[i] = motionTimeSeq[i];
		cout << "motion time(" << i << "):" << mtSeq[i] << endl;
	}
	cout << "movePTPJointAbsSeq" << endl;
	//RTC::CorbaConsumer<MotionCommands> motion = HIROControllerRtc::instance()->comp_->motion();
	//if (::CORBA::is_nil(HIROControllerRtc::instance()->comp_->motion.getObject()) == false) {
	MotionCommands::RETURN_ID * rid = HIROControllerRtc::instance()->comp_->motion()->movePTPJointAbsSeq(jpSeq);
	cout << "ret: " << rid->id << "; " << rid->comment << endl;
//	} else {
//		cout << "motion is nil." << endl;
//	}
	return error;
}

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

		if(tmp_obj.find(objname,0) != string::npos){
			return qtmp;
		}
	}
	return -1;
}
