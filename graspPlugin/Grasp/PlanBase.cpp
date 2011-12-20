// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <time.h>
#include <sys/resource.h>

#include <boost/filesystem.hpp>

#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  

//#define DEBUG_MODE
//#define SET_TOLERANCE_MODE

#include "GraspController.h"
#include "PlaceController.h"
#include "readtext.h"
#include "VectorMath.h"
#include "ForceClosureTest.h"

#include "GraspPluginManager.h"

//#define BOOST_PYTHON_STATIC_LIB
//#ifndef NDEBUG
//#define BOOST_DEBUG_PYTHON
//#endif
#include <boost/python.hpp>


#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)



namespace grasp{

double _drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}

void _initrand(){
#ifdef WIN32
	  srand((unsigned)time(NULL));
#else
	  srand48(time(0));
#endif
	}

#ifdef WIN32
double getrusage_sec() {
	return clock();
}
#else
double getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}
#endif

class PythonConverter{
	public:
		static PythonConverter* instance(){
			static PythonConverter* instance = new PythonConverter;
			return instance;
		}
		static cnoid::Vector3 setVector3(double a, double b, double c){
			return cnoid::Vector3(a,b,c);
		}
};

}

BOOST_PYTHON_MODULE( grasp )
{
using namespace boost::python;
 //   class_<grasp::GraspController, boost::noncopyable>("GraspController", no_init)
 //       .def("instance", &grasp::GraspController::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
 //   ;
    class_<grasp::PlanBase, boost::noncopyable>("PlanBase", no_init)
        .def("instance", &grasp::PlanBase::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("initial", &grasp::PlanBase::initial)
//        .def("doGraspPlanning", &grasp::PlanBase::doGraspPlanning)
 //       .def("doPlacePlanning", &grasp::PlanBase::doPlacePlanning)
  //      .def("doPickAndPlacePlanning", &grasp::PlanBase::doPickAndPlacePlanning)
		.add_property("stopFlag", &grasp::PlanBase::stopFlag) //test
    ;
	class_<cnoid::BodyItemPtr>("BodyItemPtr")
	;
	class_<cnoid::Vector3>("Vector3")
	;
    class_<grasp::PythonConverter>("PythonConverter")
		.def("Vector3", &grasp::PythonConverter::setVector3).staticmethod("Vector3")
	;
}

using namespace std;
using namespace cnoid;
using namespace grasp;


MotionState PlanBase::getMotionState(double time){
	MotionState ret;
	
	ret.jointSeq = VectorXd(body()->numJoints());
	for(int i=0;i<body()->numJoints();i++)
		ret.jointSeq[i] = body()->joint(i)->q;

	//for(int i=0;i<arm()->nJoints;i++)
	//               ret.jointSeq[arm()->arm_path->joint(i)->jointId] = arm()->arm_path->joint(i)->q;

	ret.graspingState = getGraspingState();
	ret.graspingState2 = getGraspingState2();
	ret.objectContactState = getObjectContactState();
	ret.pathPlanDOF = pathPlanDOF;
	ret.time = time;
	return ret;
}

void PlanBase::setMotionState(MotionState gm){
	for(int i=0;i<body()->numJoints();i++){
		body()->joint(i)->q = gm.jointSeq[i];
	}
	setGraspingState(gm.graspingState);
	setGraspingState2(gm.graspingState2);
	setObjectContactState(gm.objectContactState);
	pathPlanDOF = gm.pathPlanDOF;
	calcForwardKinematics();
}

PlanBase::PlanBase()  : 	os (MessageView::mainInstance()->cout() ) 
{
//	bodyItemRobot = NULL;
//	bodyItemGRC = NULL;
	targetObject = NULL;
	targetArmFinger=NULL;
	stopFlag = false;
//	refSize = refCsSize = 0;
//	arm = NULL;
//	fingers = NULL;
	graspingState = NOT_GRASPING;
	graspingState2 = NOT_GRASPING;
	tolerance = 0.0;

    if ( PyImport_AppendInittab( (char *)"grasp", initgrasp ) == -1 ) {
        MessageView::mainInstance()->put("faild init Grasp Module");
//        return;
    }
}

PlanBase::~PlanBase() {

}

PlanBase* PlanBase::instance(PlanBase *gc) {
	static PlanBase* instance = (gc) ? gc : new PlanBase();
	if(gc) instance = gc;
	return instance;
}

TargetObject::TargetObject(cnoid::BodyItemPtr bodyItem){
	bodyItemObject = bodyItem;
	object = bodyItemObject->body()->link(0);
	objVisPos = object->p;
	objVisRot = object->R;
	objMass = object->m;
	offsetApplied = false;
}


void PlanBase::SetGraspedObject(cnoid::BodyItemPtr bodyItem){
	
	targetObject = new TargetObject(bodyItem); //shoud be chaged;
	
	Box& OCP = targetObject->OCP;
	calcBoundingBox(object()->coldetModel, OCP.edge, OCP.p, targetObject->objCoM_, OCP.R);
	if(targetArmFinger){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
		arm()->palmObjPair = new ColdetLinkPair(palm(),object() );
	}
	string tagId = bodyItem->name();
	if(objTag2Item.find(tagId) == objTag2Item.end()){ 
		objTag2Item.insert( pair <string,BodyItemPtr>(tagId, bodyItem) );
	}
	setObjectContactState(ON_ENVIRONMENT) ;
	
}

ArmFingers::ArmFingers(cnoid::BodyItemPtr bodyItem, const YamlMapping& gSettings) :  os (MessageView::mainInstance()->cout() ) 
{
	bodyItemRobot = bodyItem;

	boost::filesystem::path robotfullpath(bodyItemRobot->modelFilePath());
	bodyItemRobotPath = boost::filesystem::path (robotfullpath.branch_path()).string();
	dataFilePath = bodyItemRobotPath + "/data/";
	
	//READ YAML setting

	palm = bodyItemRobot->body()->link(gSettings["palm"].toString());
	base = bodyItemRobot->body()->link(gSettings["base"].toString());

	const YamlSequence& tips = *gSettings["fingerEnds"].toSequence();

	nFing = tips.size();
	fingers = new FingerPtr[nFing];

	arm=NULL;
	for (int i = 0;i < tips.size();i++) {
		fingers[i]=NULL;
	}

	if( gSettings.find("GrasplotPluginDir")->type() == YAML_SCALAR ){
		string pluginPath = bodyItemRobotPath + "/" + gSettings["GrasplotPluginDir"].toString();
		os << "Grasplot Plugin Path " << pluginPath << endl;
		gPluginManager.scanPluginFiles(pluginPath);

		arm = (Arm *)gPluginManager.loadGrasplotPlugin(bodyItemRobot->body(),base,palm, "Arm");
		
		for (int i = 0;i < tips.size();i++) {
			if(!fingers[i]) fingers[i] = (Finger *)gPluginManager.loadGrasplotPlugin
					(bodyItemRobot->body(), palm, bodyItemRobot->body()->link(tips[i].toString()), "Finger");
		}
	}
	if(!arm){
		arm = new Arm(bodyItemRobot->body(),base, palm);
	}
	for (int i = 0;i < tips.size();i++) {
		if(!fingers[i]) fingers[i] = new Finger(bodyItemRobot->body(), palm, bodyItemRobot->body()->link(tips[i].toString()) );
		fingers[i]->number = i;
	}
	
	if( gSettings.find("dataFilePath")->type() == YAML_SCALAR ){ 
		dataFilePath = bodyItemRobotPath + "/" + gSettings["dataFilePath"].toString() +"/";
	}

	if( gSettings.find("armStandardPose")->type() == YAML_SEQUENCE ){ 
		const YamlSequence& list = *gSettings["armStandardPose"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->armStandardPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("fingerOpenPose")->type() == YAML_SEQUENCE ){ 
		const YamlSequence& list = *gSettings["fingerOpenPose"].toSequence();
		int j=0;
		int k=0;
		for(int i=0;i<list.size();i++){
			if(i>=k+fingers[j]->fing_path->numJoints()){
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPose.push_back(list[i].toDouble());
		}
	}

	if( gSettings.find("pythonInterface")->type() == YAML_SCALAR ){ 
		pythonInterface = bodyItemRobotPath + "/" + gSettings["pythonInterface"].toString();
	}else{
		pythonInterface = "/NULL";
	}
	
	vector <InterLink> & interLinkList = PlanBase::instance()->interLinkList;
	if( gSettings.find("interlink")->type() == YAML_SEQUENCE ){ 
		const YamlSequence& list = *gSettings["interlink"].toSequence();
		for(int i=0;i<list.size();i++){
			const YamlSequence& ilist = *list[i].toSequence();
			Link* master = bodyItemRobot->body()->link(ilist[0].toString());			
			double baseratio = ilist[1].toDouble();			
			for(int j=1;j<ilist.size()/2;j++){
				InterLink temp;
				temp.master = master;
				temp.slave = bodyItemRobot->body()->link(ilist[2*j].toString());
				temp.ratio = ilist[2*j+1].toDouble()/baseratio;
				interLinkList.push_back(temp);
			}
			
		}
	}
	if( gSettings.find("approachOffset")->type() == YAML_SEQUENCE ){ 
		const YamlSequence& list = *gSettings["approachOffset"].toSequence();
		for(int i=0;i<list.size();i++){
			arm->approachOffset[i] = list[i].toDouble();
		}
	}

	if( gSettings.find("selfContactPair")->type() == YAML_SEQUENCE ){ 
		const YamlSequence& list = *gSettings["selfContactPair"].toSequence();
		for(int i=0;i<list.size()/2;i++){
			contactLinks.insert ( make_pair ( list[i*2].toString(), list[i*2+1].toString() ) );
		}
	}

	handJoint = new LinkTraverse(palm);
	nHandLink = handJoint->numLinks();
	
	if (gSettings.find("name")->type() == YAML_SCALAR ){
		name = gSettings["name"].toString();
	}else{
		static int i=0;
		stringstream namenum;
		namenum << arm << i;
		name = namenum.str();
	}

}

bool PlanBase::SetGraspingRobot(cnoid::BodyItemPtr bodyItem_){
	//setting robot

	armsList.clear(); //Temoporary;  will delete menbers
	interLinkList.clear();
	targetArmFinger = NULL;
	
	//READ YAML setting
	if( bodyItem_->body()->info()->find("graspPluginSetting")->type() == YAML_SEQUENCE){ // multi arm
		const YamlSequence& glist = *(*bodyItem_->body()->info())["graspPluginSetting"].toSequence();
		for(int i=0;i<glist.size();i++){
			const YamlMapping& gSettings = *glist[i].toMapping();
			if ( gSettings.isValid() && !gSettings.empty()) {
				targetArmFinger = new ArmFingers(bodyItem_, gSettings);
				armsList.push_back(targetArmFinger);
			}
		}
	}
	else{ // single arm
		const YamlMapping& gSettings = *bodyItem_->body()->info()->findMapping("graspPluginSetting");
		if ( gSettings.isValid() && !gSettings.empty()) {
			targetArmFinger = new ArmFingers(bodyItem_, gSettings);
			armsList.push_back(targetArmFinger);
		}
	}

	targetArmFinger = armsList[0];
	
	if(targetArmFinger == NULL){
		os << "ERROR graspPluginSetting is not found in yaml" << endl;
		return false;
	}
	
	robTag2Arm.clear();	
	for(int i=0;i<armsList.size();i++){
		armsList[i]->id = i;
		string tagId = armsList[i]->name;
		if(robTag2Arm.find(tagId) != robTag2Arm.end()){ 
			os << "Error: the tagId is already recorded " << tagId << endl;
			continue;
		}else{
			robTag2Arm.insert( pair <string,ArmFingers*>(tagId, armsList[i]));
		}
	}
	
	os  << bodyItem_->name() << " has " <<armsList.size() << " arm(s) "<< endl;

	if(targetObject){
		for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
		arm()->palmObjPair = new ColdetLinkPair(palm(),object() );
	}

	bodyItemRobot()->body()->calcForwardKinematics();
	
	graspMotionSeq.clear();
	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);

	return true;
}


bool PlanBase::flush(){
	static int cnt=0;
	cnt++;

	if(stopFlag){
		stopFlag=false;
		throw(cnt);
	}
/* it will be GraspController	
	if(bodyItemGRC){
		bodyItemGRC->body()->link(0)->R = palm()->R*(GRCmax.R);
		bodyItemGRC->body()->link(0)->p = palm()->p+palm()->R*GRCmax.p;
		bodyItemGRC->notifyKinematicStateChange();
	}
*/
//	bodyItemRobot()->body()->calcForwardKinematics();
	bodyItemRobot()->notifyKinematicStateChange();
	targetObject->bodyItemObject->notifyKinematicStateChange();
	MessageView::mainInstance()->flush();

#ifdef  DEBUG_MODE	
	usleep(100000);
#endif
	return true;
	
}

void PlanBase::calcForwardKinematics(){
	
	setInterLink();

	bodyItemRobot()->body()->calcForwardKinematics();

	if(graspingState==GRASPING) {
		object()->R = palm()->R*(targetArmFinger->objectPalmRot);
		object()->p = palm()->p+palm()->R*targetArmFinger->objectPalmPos;
	}
	else if(graspingState2==GRASPING) {
		object()->R = palm(1)->R*(armsList[1]->objectPalmRot);
		object()->p = palm(1)->p+palm(1)->R*armsList[1]->objectPalmPos;
	}
}

bool PlanBase::isColliding(){
//	cnoid::ColdetLinkPairPtr* robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs;
	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
		testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	for(int i=0;i<robotEnvPairs.size();i++){
		ColdetLinkPairPtr testPair = robotEnvPairs[i];
		testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
		testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"robot env collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return true;
		}
	}
	if(checkAllGraspingState()==NOT_GRASPING){
		for(int i=0;i<robotObjPairs.size();i++){
			ColdetLinkPairPtr testPair = robotObjPairs[i];
			testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
			testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}
	}
	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
			testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
			bool coll = testPair->checkCollision();
			if(coll){
#ifdef DEBUG_MODE
				cout <<"collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}
	}
	
	return false;
}

double PlanBase::clearance(){

//	double start = getrusage_sec();
	
	double min_sep=1.e10;

	for(int i=0;i<robotSelfPairs.size();i++){
		ColdetLinkPairPtr testPair = robotSelfPairs[i];
		testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
		testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
		bool coll = testPair->checkCollision();
		if(coll){
			colPairName[0] = testPair->model(0)->name();
			colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
	}
	if(checkAllGraspingState()==NOT_GRASPING){
		for(int i=0;i<robotObjPairs.size();i++){
			ColdetLinkPairPtr testPair = robotObjPairs[i];
			testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
			testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);
			bool coll = testPair->checkCollision();
			if(coll){
				colPairName[0] = testPair->model(0)->name();
				colPairName[1] = testPair->model(1)->name();
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return 0;
			}
		}
	}

	
	for(int i=0;i<robotEnvPairs.size();i++){
		ColdetLinkPairPtr testPair = robotEnvPairs[i];
		testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
		testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);

		testPair->setTolerance(tolerance);
		if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return 0;
		}
		continue;

		int t1,t2;
		double p1[3],p2[3];
		double distance = testPair->computeDistance(t1,p1,t2,p2);
		if(distance <=tolerance){
#ifdef DEBUG_MODE
			os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
			return distance;
		}
		else {
			os << distance << endl;
		}
		if(distance < min_sep){
			min_sep = distance;
		}
	}


	if(getObjectContactState()==OFF_ENVIRONMENT){
		for(int i=0;i<objEnvPairs.size();i++){
			ColdetLinkPairPtr testPair = objEnvPairs[i];
			testPair->model(0)->setPosition(testPair->link(0)->R, testPair->link(0)->p);
			testPair->model(1)->setPosition(testPair->link(1)->R, testPair->link(1)->p);

			testPair->setTolerance(tolerance);
			if( testPair->detectIntersection() ){
#ifdef DEBUG_MODE
				os <<"rob-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return 0;
			}
			continue;

			int t1,t2;
			double p1[3],p2[3];
			double distance = testPair->computeDistance(t1,p1,t2,p2);
			if(distance <=tolerance){
#ifdef DEBUG_MODE
				os <<"obj-env tolerance collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return distance;
			}
			else {
				os << distance << endl;
			}
			if(distance < min_sep){
				min_sep = distance;
			}
		}
	}
//	double end = getrusage_sec();
//	cout << "time clearance" << objEnvPairs.size() << " "<< end - start << endl;

	
	return min_sep;
}
	

void PlanBase::setGraspingState(int state){
	if(state==GRASPING){
		targetArmFinger->objectPalmPos = trans(palm()->R)*(object()->p - palm()->p);	
		targetArmFinger->objectPalmRot = trans(palm()->R)*object()->R ;	
	}
	graspingState = state;
}

void PlanBase::setGraspingState2(int state){
	if(armsList.size() >1 && state==GRASPING){
		armsList[1]->objectPalmPos = trans(palm(1)->R)*(object()->p - palm(1)->p);	
		armsList[1]->objectPalmRot = trans(palm(1)->R)*object()->R ;	
	}
	graspingState2 = state;
}

void PlanBase::setTrajectoryPlanDOF(){
	
	pathPlanDOF.clear();

	for(int i=0; i<arm()->nJoints; i++)
		pathPlanDOF.push_back(arm()->arm_path->joint(i)->jointId);
	for(int i=0; i<nFing(); i++)
		for(int j=0; j<fingers(i)->nJoints; j++)
			pathPlanDOF.push_back(fingers(i)->fing_path->joint(j)->jointId);

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

void PlanBase::setTrajectoryPlanDOF(int k){
	
	pathPlanDOF.clear();

	for(int i=0; i<arm(k)->nJoints; i++)
		pathPlanDOF.push_back(arm(k)->arm_path->joint(i)->jointId);
	for(int i=0; i<nFing(k); i++)
		for(int j=0; j<fingers(k,i)->nJoints; j++)
			pathPlanDOF.push_back(fingers(k,i)->fing_path->joint(j)->jointId);

#ifdef DEBUG_MODE
	cout << "Plan DOF= ";
	for (unsigned int i=0; i<pathPlanDOF.size(); i++)
		cout << pathPlanDOF[i] << " ";	cout << endl;
#endif
}

double PlanBase::calcContactPoint(ColdetLinkPairPtr cPair, Vector3 &Po, Vector3 &Pf, Vector3 &objN) {

//	int Ik = t;

	double p1[3] = {0}, p2[3] = {0};
	int tid1, tid2;

//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
//	ColdetModelPtr model = cPair->model(0);

	cPair->model(0)->setPosition(cPair->link(0)->R, cPair->link(0)->p);
	cPair->model(1)->setPosition(cPair->link(1)->R, cPair->link(1)->p);
//	cout << cPair->link(0)->p << cPair->link(1)->p << endl;

	double dsn = cPair->computeDistance(tid1, &p1[0], tid2, &p2[0]);


	Link* links[2];
	links[0] = cPair->link(0);
	links[1] = cPair->link(1);

	int v[2][3];
	links[0]->coldetModel->getTriangle(tid1, v[0][0], v[0][1], v[0][2]);
	links[1]->coldetModel->getTriangle(tid2, v[1][0], v[1][1], v[1][2]);

	float p[3];
	Vector3 n[3];

	for (int i = 1; i < 2;i++) {
		for (int j = 0; j < 3;j++) {
			links[i]->coldetModel->getVertex(v[i][j], p[0], p[1], p[2]);
			n[j] = Vector3 (p[0], p[1], p[2]);
		}
	}

	Pf = Vector3(p1[0], p1[1], p1[2]);
	Po = Vector3(p2[0], p2[1], p2[2]);
//	cout << Po << Pf << endl;
	
	Po = trans(cPair->link(1)->R) * Po - cPair->link(1)->p;
//	alias(Pf) = trans(cPair->link(0)->R) * Pf - cPair->link(0)->p;

	
	Vector3 objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	objN = objN2 / norm2(objN2);

	return dsn;

}


void PlanBase::calcBoundingBox(ColdetModelPtr model, Vector3 &edge, Vector3& center, Vector3& com, Matrix3& Rot) {
	//objVis and objPos shoulde be defined in advance.

	class Triangle{
	public:
		cnoid::Vector3 ver[3];
		float area;
	};

	// convert coldetmodel to objectshape
	float out_x, out_y, out_z;
	int v0,v1,v2;
	
	int nVerticies = model->getNumVertices();
	int nTriangles = model->getNumTriangles();
	
	Vector3* verticies = new Vector3[nVerticies];
	Triangle* triangles = new Triangle[nTriangles];
	
	for(int i=0;i<nVerticies;i++){
        model->getVertex(i, out_x, out_y, out_z);
		verticies[i][0] = out_x;
		verticies[i][1] = out_y;
		verticies[i][2] = out_z;
	}

	for(int i=0;i<nTriangles;i++){
        model->getTriangle(i, v0,v1,v2);
		triangles[i].ver[0] = verticies[v0];
		triangles[i].ver[1] = verticies[v1];
		triangles[i].ver[2] = verticies[v2];
	}
	
	// calc distribution
	Vector3 pt;
	MatrixXd distribute = MatrixXd::Zero(3, 3);
	Vector3 average(0, 0, 0);
	
	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1] - triangles[i].ver[0]);
		Vector3 e2 (triangles[i].ver[2] - triangles[i].ver[0]);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
	}

	Matrix3 aq;
	Vector3 sumCenter(0,0,0);
	double sumArea=0;
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) aq(i,j)=0;

	for(int l=0;l<nTriangles;l++){
		Triangle& t = triangles[l];
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					aq(j,k) += t.area*(t.ver[i][j] * t.ver[i][k])/6.0;
					aq(j,k) += t.area*(t.ver[i][j] * t.ver[(i+1)%3][k])/12.0;
					aq(j,k) += t.area*(t.ver[(i+1)%3][j] * t.ver[i][k])/12.0;
				}
			}
		}
		sumArea +=t.area;
		sumCenter  =  sumCenter + t.area/3.0* Vector3 ( t.ver[0] + t.ver[1] + t.ver[2]);
	}
	average = com = sumCenter/sumArea;
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = aq(j,k) - com[j] * com[k] * sumArea;
		}
	}
	MatrixXd evec(3, 3);
	VectorXd eval(3);
	int info;
	info = calcEigenVectors(distribute, evec, eval);

	Vector3 e[3];

	
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			e[j][i] = evec(i, j);
	

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(int l=0;l<nVerticies;l++){
		Vector3 pt = verticies[l];
		for (int j = 0; j < 3; j++) {
			double tmp = dot(e[j], Vector3(pt - average));
			if (tmp > pt_max[j]) pt_max[j] = tmp;
			if (tmp < pt_min[j]) pt_min[j] = tmp;
		}
	}

	Rot =  (d2v(evec));

	edge =  (pt_max - pt_min);
	center =  average + 0.5 * (pt_max + pt_min);
//	com = average;
	
//	alias(Rot)  = objVisRot() * Rot;
//	alias(center) = objVisRot() * center + objVisPos();
//	alias(com)    = objVisRot() * com   + objVisPos();
	cout << edge << Rot ;
	
	
	delete []	verticies;
	delete []  triangles;


	return;
}




//Including Enveloping grasp
//bool PlanBase::sampleFinalPos2(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, int iterate)
bool PlanBase::initial() {
	if ( !targetObject || !targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}
	
	RemoveEnvironment(targetObject->bodyItemObject);
	
	targetObject->objVisPos = object()->p;
	targetObject->objVisRot = object()->R;

	setGraspingState(NOT_GRASPING);
	setGraspingState2(NOT_GRASPING);
	graspMotionSeq.push_back ( getMotionState() );
	
	initialCollision();

	_initrand();
	return true;
}

void PlanBase::initialCollision(){

	robotSelfPairs.clear();
	robotEnvPairs.clear();
	robotObjPairs.clear();
	objEnvPairs.clear();

	if(targetArmFinger==NULL) {
		return;
	}
/*	for(unsigned int i=0; i<arm()->arm_path->numJoints(); i++){
		for(unsigned int j=i+2; j<arm()->arm_path->numJoints(); j++){
			robotSelfPairs.push_back(new ColdetLinkPair(arm()->arm_path->joint(i), arm()->arm_path->joint(j)) );
		}
	}
	
*/
	for(unsigned int i=0;i<bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as 
		for(unsigned int j=i+2;j<bodyItemRobot()->body()->numJoints();j++){
			bool pass = false;
			pair<multimap<string, string>::iterator, multimap<string, string>::iterator> ppp;
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->joint(i)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->joint(j)->name()) pass = true;
			}
			ppp = targetArmFinger->contactLinks.equal_range(bodyItemRobot()->body()->joint(j)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == bodyItemRobot()->body()->joint(i)->name()) pass = true;
			}
			if(pass) continue;
			ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->joint(i),bodyItemRobot()->body()->joint(j));
			temp->model(0)->setPosition(temp->link(0)->R, temp->link(0)->p);
			temp->model(1)->setPosition(temp->link(1)->R, temp->link(1)->p);
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-04)	robotSelfPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at robotSelfPair"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl; 
#endif
		}
	}
	for(unsigned int j=0;j<bodyItemRobot()->body()->numJoints();j++){
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
				ColdetLinkPairPtr temp= new ColdetLinkPair(bodyItemRobot()->body()->joint(j), (*it)->body()->link(i));
				temp->model(0)->setPosition(temp->link(0)->R, temp->link(0)->p);
				temp->model(1)->setPosition(temp->link(1)->R, temp->link(1)->p);
				int t1,t2;
				double p1[3],p2[3];
				double distance = temp->computeDistance(t1,p1,t2,p2);
				if(distance>1.0e-04)	robotEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
				else os <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl; 
#endif
			}
		}
	}
/*	for(unsigned int j=0;j<arm()->arm_path->numJoints();j++){
		for( list<BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end(); it++){
			ColdetLinkPairPtr temp= new ColdetLinkPair(arm()->arm_path->joint(j), (*it)->body()->link(0));
			temp->model(0)->setPosition(temp->link(0)->R, temp->link(0)->p);
			temp->model(1)->setPosition(temp->link(1)->R, temp->link(1)->p);
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-03)	armEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"tollerance collide on initial condition"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl; 
#endif
			os <<"tollerance collide on initial condition"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl; 
		}
	}
*/	if(targetObject){
		for(unsigned int j=0;j<bodyItemRobot()->body()->numJoints();j++){
			robotObjPairs.push_back(new ColdetLinkPair(bodyItemRobot()->body()->joint(j), object() ));
	  	}
		for(list<cnoid::BodyItemPtr>::iterator it = bodyItemEnv.begin(); it !=bodyItemEnv.end();it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
				objEnvPairs.push_back(new ColdetLinkPair(object(), (*it)->body()->link(i)));
			}
		}
	}
}



void PlanBase::setObjPos(const cnoid::Vector3& P, const cnoid::Matrix3 R){

	targetObject->objVisPos = P;
	targetObject->objVisRot = R;
	object()->p=P;
	object()->R=R;

	targetObject->offsetApplied = false;

	return;

}

void PlanBase::setVisOffset(const cnoid::Vector3& P)
{
	if(!targetObject->offsetApplied){
		targetObject->objVisPos += P;
		object()->p += P;
		targetObject->offsetApplied = true;
	}
}

void PlanBase::setVisOffsetR()
{
	Vector3 y = rpyFromRot(targetObject->objVisRot);
	if( fabs(y(0))<0.1 ) y(0)=0;
	else if(fabs(y(0)-1.5708)<0.1 ) y(0) =  1.5708;
	else if(fabs(y(0)+1.5708)<0.1 ) y(0) = -1.5708;
	else if(fabs(y(0)-3.1415)<0.1 ) y(0) =  3.1415;
	else if(fabs(y(0)+3.1415)<0.1 ) y(0) = -3.1415;

	if( fabs(y(1))<0.1 ) y(1)=0;
	else if(fabs(y(1)-1.5708)<0.1 ) y(1) =  1.5708;
	else if(fabs(y(1)+1.5708)<0.1 ) y(1) = -1.5708;
	else if(fabs(y(1)-3.1415)<0.1 ) y(1) =  3.1415;
	else if(fabs(y(1)+3.1415)<0.1 ) y(1) = -3.1415;

	targetObject->objVisRot = rotFromRpy(y);
	object()->R = rotFromRpy(y);
}

void PlanBase::removeVisOffset(const cnoid::Vector3& P)
{
	if(targetObject->offsetApplied){
		targetObject->objVisPos -= P;
		object()->p -= P;
		targetObject->offsetApplied = false;
	}
}

void PlanBase::setInterLink(){
	if(interLinkList.empty()) return;
	for(int i=0; i<interLinkList.size();i++){
		interLinkList[i].slave->q = interLinkList[i].master->q *interLinkList[i].ratio;
		if( interLinkList[i].slave->q < interLinkList[i].slave->llimit) interLinkList[i].slave->q = interLinkList[i].slave->llimit; 
		if( interLinkList[i].slave->q > interLinkList[i].slave->ulimit) interLinkList[i].slave->q = interLinkList[i].slave->ulimit; 
	}
}
