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

#include <cnoid/JointPath>
#include <cnoid/MessageView>

#define DEBUG_MODE

#include "GraspController.h"
#include "readtext.h"
#include "VectorMath.h"
#include "ForceClosureTest.h"

#include "GraspPluginManager.h"


#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)


using namespace std;
using namespace cnoid;
using namespace grasp;



GraspController::GraspController()  : 	os (MessageView::mainInstance()->cout() ) 
{
	bodyItemGRC = NULL;
	targetObject = NULL;
	targetArmFinger=NULL;
	refSize = refCsSize = 0;
}

GraspController::~GraspController() {

}

GraspController* GraspController::instance(GraspController *gc) {
	static GraspController* instance = (gc) ? gc : new GraspController();
	if(gc) instance = gc;
	return instance;
}


double GraspController::calcContactPoint(ColdetLinkPairPtr cPair, Vector3 &Po, Vector3 &Pf, Vector3 &objN) {

//	int Ik = t;

	double p1[3] = {0}, p2[3] = {0};
	int tid1, tid2;

//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
//	ColdetModelPtr model = cPair->model(0);

	cPair->model(0)->setPosition(cPair->link(0)->R, cPair->link(0)->p);
	cPair->model(1)->setPosition(cPair->link(1)->R, cPair->link(1)->p);

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
	
	Po = trans(cPair->link(1)->R) * Po - cPair->link(1)->p;
	
	Vector3 objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	objN = objN2 / norm2(objN2);

	return dsn;

}


//Including Enveloping grasp
//bool GraspController::sampleFinalPos2(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs, int iterate)
bool GraspController::sampleFinalPos(int iterate) {

	Homogenous Palm;
	Palm.p = palmPos;
	Palm.R = palmRot;
	
	vector <Vector3* > link_org_p(nFing());
	vector <Matrix3*> link_org_R(nFing());
	vector <double* > link_org_q(nFing());

	arm()->IK_arm(Palm.p, Palm.R);
	bodyItemRobot()->body()->calcForwardKinematics();
	

	for (int i = 0; i < nFing(); i++) {
		link_org_p[i] = new Vector3[ fingers(i)->nJoints ];
		link_org_R[i] = new Matrix3[ fingers(i)->nJoints ];
		link_org_q[i] = new double[ fingers(i)->nJoints ];
		for (int j = 0; j < fingers(i)->nJoints; j++) {
			link_org_p[i][j] = fingers(i)->fing_path->joint(j)->p;
			link_org_R[i][j] = fingers(i)->fing_path->joint(j)->R;
			link_org_q[i][j] = fingers(i)->fing_path->joint(j)->q;
		}
	}
	
	int nContact=0;
	
	if(arm()->palmContact) nContact++;	
	for(int i=0;i<nFing();i++) for(int j=0;j<fingers(i)->nJoints;j++) if(fingers(i)->contact[j]) nContact++;

	vector <Vector3> objPos(nContact), objN(nContact);

	bool ik_solved;

	for (int times = 0; times < iterate; times++) {

		Vector3 dPos((_drand() - 0.5)*dif(0), 0.8*(_drand() - 0.5)*dif(1), (_drand() - 0.5)*dif(2) );
		if(!times) dPos = Vector3(0,0,0);

		ik_solved = arm()->IK_arm(Vector3(Palm.p + Palm.R * GRCmax.R * dPos), Palm.R);
		if(!ik_solved) continue;

		for(int i=0;i<nFing();i++){
			for (int j = 0; j < fingers(i)->nJoints; j++) {
				fingers(i)->fing_path->joint(j)->q = link_org_q[i][j] ;
			}
		}
		
		bodyItemRobot()->body()->calcForwardKinematics();

		int cnt=0;
		if(arm()->palmContact){
			if( arm()->closeArm(0,1000,objPos[cnt], objN[cnt]) ){
				cnt++;
			}
			else continue;
		}
		
		for(int i=0;i<nFing();i++){
			fingers(i)->contactSearch(cnt, 1000, &objPos[0], &objN[0]);
			tc->flush() ;
		}		
		
		if(cnt < nContact) continue;

		VectorXd wrench = VectorXd::Zero(6);
		//wrench(2)   = -objMass*9.8;

		double Qtmp, q_threshold = 0.01;
		Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoid(wrench, &objPos[0], &objN[0],nContact, 0.5, 5.0);
		
		cout <<"force closure" << Qtmp << endl;
		
		if(cnt < 3 || nContact<3){	Qtmp=1; cout<<  "ForceClosureTest>0 for gripper" << endl; };

		if (Qtmp > q_threshold) {
			palmPos = palm()->p;
			palmRot = Palm.R;
			arm()->IK_arm(palmPos, palmRot);
			bodyItemRobot()->body()->calcForwardKinematics();
			return true;
		}
	}//end of main loop

	return false;
}







void GraspController::readMotionFile(const char argv[]) {

	string pos_data   = tc->dataFilePath() + argv + ".pos";

	ifstream fin_pos(pos_data.c_str());

	double time;

	while (!fin_pos.eof()) {
		fin_pos >> time;
		for (int i = 1;i < nHandLink();i++)	fin_pos >> handJoint()->link(i)->q;
	}

	return;
}


double GraspController::returnRefMotion(bool cs, string refMotion_) {
	//Heuristic rule to define the grasp pattern. Before running this function,
	//1. getObjShape should be run
	//2. objMass should be defined

	Vector3 oE = sort(targetObject->OCP_edge());
	bool flag = false;

	string refMotion_selected = "";

	string prm_data = tc->dataFilePath() + refMotion_ + ".prm";
	
	ifstream fin_prm(prm_data.c_str());

#ifdef DEBUG_MODE	
	cout << prm_data << endl;
#endif
	
	parseParameters(fin_prm);
	


	Vector3 hBE( sort(GRCmax.edge) );
	Vector3 hDE( sort(GRCdes.edge) );

	if (cs) {
		GRCmax.edge[2] = GRCmax.edge[2]*1.5; 
		double crx0 = sqrt( dbl(hBE[0]) + dbl(hBE[1]) ), crx1=hBE[2];
		hBE = Vector3(crx0, crx0, crx1);
	}

	double sb = min(Vector3(hBE - oE));
	double sd = norm2(Vector3(hDE - oE));

	refMotion_selected = refMotion_;

	if ( (targetObject->objMass < maxLoad)  && (sb > 0.0) && (targetObject->objMass > minLoad) ) {
		flag = true;
		cout << refMotion_selected << " is candidate " << endl;
	}
	if(!flag) return -1;
	return sd ;
}


Matrix3 GraspController::calcObjRot(const Matrix3 &a, int n) {
	Matrix3 ret;

	for (int i = 0; i < 3; i++) {
		if (n == 0) {    ret(i, 0) = a(i, 0);     ret(i, 1) = a(i, 1);     ret(i, 2) = a(i, 2);     }
		else if (n == 1) {    ret(i, 0) = a(i, 2);     ret(i, 1) = a(i, 0);     ret(i, 2) = a(i, 1);     }
		else if (n == 2) {    ret(i, 0) = a(i, 1);     ret(i, 1) = a(i, 2);     ret(i, 2) = a(i, 0);     }
		else if (n == 3) {    ret(i, 0) = -a(i, 1);     ret(i, 1) = -a(i, 0);     ret(i, 2) = -a(i, 2);     }
		else if (n == 4) {	ret(i, 0) = -a(i, 2);     ret(i, 1) = -a(i, 1);     ret(i, 2) = -a(i, 0);     }
		else if (n == 5) {	ret(i, 0) = -a(i, 0);     ret(i, 1) = -a(i, 2);     ret(i, 2) = -a(i, 1);     }
	}

	return ret;
}

Matrix3 GraspController::calcHandRot(Matrix3 &a1, int n1) {
	Matrix3 ret;
	Vector3 x1(0, 0, 0);
	Vector3 y1(0, 0, 0);
	Vector3 z1(0, 0, 0);

//    for(int i=0; i<3; i++) z1[i]= -1.0*a1(i,2);
	z1 = -1.0 * col(a1, 2);

	if (n1 == 0)      x1 =      col(a1, 0);
	else if (n1 == 1)      x1 = -1.0 * col(a1, 0);
	else if (n1 == 2)      x1 =      col(a1, 1);
	else if (n1 == 3)      x1 = -1.0 * col(a1, 1);

	y1 = cross(z1, x1);

	for (int i = 0; i < 3; i++) {
		ret(i, 0) = x1(i);
		ret(i, 1) = y1(i);
		ret(i, 2) = z1(i);
	}
//	cout <<n1 << a1 << x1 << y1 << z1 << ret;

	return ret;
}

double GraspController::getPalmPosture() {
	// Heuristic rule to define the final posture. Before running this function,
	// selectRefMotion should be run.
	//Box& OCP = targetObject->OCP;
	Box OCP;
	OCP.R = targetObject->OCP_R();
	OCP.edge = targetObject->OCP_edge();
	OCP.p = targetObject->OCP_p();

	double eval = 1.e10;

#ifdef DEBUG_MODE	
	ofstream fout_log("palmPos.log");
	fout_log << "OCP_Pos" << OCP.p.transpose() << endl;
	fout_log << "OCP_Rot" << rpyFromRot(OCP.R).transpose() << endl;
	fout_log << "OCP_Edge" << OCP.edge.transpose() << endl;
#endif

	Homogenous relGRCface, GRCface, OCPface;
	Homogenous Palm;

	relGRCface.p << 0, 0, 0;
	relGRCface.R << 1, 0, 0,  0, 0, 1,  0, -1, 0 ;

	Vector3 ex(1, 0, 0);
	Vector3 ey(0, 1, 0);
	Vector3 ez(0, 0, 1);

	//==For six faces of OCP_==
	for (int i = 0; i < 6; i++) {
		Homogenous OCPface_;
		OCPface_.R = calcObjRot(OCP.R, i);

		//==For four rotation angles about OCP_ normal==
		for (int j = 0; j < 4; j++) {

			GRCface.R = calcHandRot(OCPface_.R, j);

			Vector3 d0 ( relGRCface.R * trans(GRCface.R) * OCP.R * OCP.edge );

			Palm.R = GRCface.R * (GRCmax.R * relGRCface.R).transpose();

			if(0.5*fabs(d0(1)) + dot(GRCmax.R*ey, GRCmax.p-GRCdes.p) < 0.5*GRCmax.edge(1))
				Palm.p = OCP.p + Palm.R * dGRC_Pos_ - Palm.R * GRCdes.p;
			else{
				Vector3 ap(Palm.R*GRCmax.R*ey);
				Palm.p = OCP.p + Palm.R * dGRC_Pos_ + 0.5*(GRCmax.edge(1) - fabs(d0(1)))*ap - Palm.R * GRCmax.p;
			}
			
#ifdef DEBUG_MODE	
			palm()->p = Palm.p;
			palm()->R = Palm.R;
//			tc->flush();
//			os << d0 << GRCmax.edge  << endl;
#endif
	
			if( fabs(d0[2])>GRCmax.edge[2] ) continue;
			if( fabs(d0[1])>GRCmax.edge[1] ) continue;
			if( fabs(d0[0])>GRCmax.edge[0] ) continue;
			if( fabs(d0[0])<GRCmin.edge[0] ) continue;
	
			bool ikConv = arm()->IK_arm(Palm.p, Palm.R);
			double leng = norm2(Palm.p - arm()->arm_path->joint(0)->p);
			double quality = arm()->avoidAngleLimit();
			bool limit =  arm()->checkArmLimit();

#ifdef DEBUG_MODE	
			fout_log << endl;
			fout_log << leng <<" "<< (eval>quality) <<" "<< quality <<" "<< ikConv <<" "<< limit <<" "<< !tc->isColliding() << endl;
			fout_log << "OCPface" << rpyFromRot(OCPface_.R).transpose() << endl;
			fout_log << "GRCface" << rpyFromRot(GRCface.R).transpose() << endl;
			fout_log << "palmPos" << Palm.p.transpose() << endl;
			fout_log << "palmRot" << rpyFromRot(Palm.R).transpose() << endl;
			for (int k = 0; k < arm()->arm_path->numJoints(); k++) fout_log << arm()->arm_path->joint(k)->q << " "; fout_log << endl;
			if(tc->isColliding())fout_log << "Collision " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;
#endif

			if ( (eval > quality) && ikConv && limit && !tc->isColliding()) {
//			if ( leng < 3 &&  (eval > quality) && ikConv && limit ) {
				palmPos   = Palm.p;
				palmRot   = Palm.R;
				OCPface.R = OCPface_.R;
				eval = quality;
				os << "Found feasible palm posture " << endl;

#ifdef DEBUG_MODE	
				fout_log <<  "Found feasible palm posture " << endl;
#endif
			}
		}
	}
	
	dif = Vector3 ( fabs( (trans(Matrix3(palmRot*GRCmax.R))*OCP.R*OCP.edge)[0] ) - GRCdes.edge[0],
	        fabs( dot(Vector3(0, 0, 1), trans(OCPface.R)*OCP.R*OCP.edge) ),
	        -fabs( (trans(Matrix3(palmRot*GRCmax.R))*OCP.R*OCP.edge)[2] ) + fabs( GRCmax.edge[2]) );
	

#ifdef DEBUG_MODE	
	fout_log.close();
#endif

	return eval;
}

bool GraspController::initial(TargetObject* targetObject, ArmFingers* targetArmFinger)
 {
	if ( !targetObject || !targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}
	this->targetObject = targetObject;
	this->targetArmFinger = targetArmFinger;
	
	tc = PlanBase::instance();
	tc->RemoveEnvironment(targetObject->bodyItemObject);
	
	targetObject->objVisPos = object()->p;
	targetObject->objVisRot = object()->R;
	
	tc->initialCollision();

	//READ YAML setting
	YamlMapping* gSettings;
	if( bodyItemRobot()->body()->info()->find("graspPluginSetting")->type() == YAML_SEQUENCE){
		gSettings = (*(*bodyItemRobot()->body()->info())["graspPluginSetting"].toSequence())[0].toMapping();
	}
	else{
		gSettings  = bodyItemRobot()->body()->info()->findMapping("graspPluginSetting");
	}
	
	if (gSettings->isValid() && !gSettings->empty()) {
		refSize =0;
		if( gSettings->find("prehensionList")->type() == YAML_SEQUENCE ){ 
			const YamlSequence& plist = *(*gSettings)["prehensionList"].toSequence();
			refSize = plist.size();
			if(refSize) reffile = new string[refSize];
			for (int i = 0;i < refSize; i++) {
				reffile[i] = plist[i].toString(); 
			}
		}
		refCsSize = 0;
		if( gSettings->find("prehensionCsList")->type() == YAML_SEQUENCE ){ 
			const YamlSequence& pcslist = *(*gSettings)["prehensionCsList"].toSequence();
			refCsSize = pcslist.size();
			if(refCsSize) refCsfile = new string[refCsSize];
			for (int i = 0;i < refCsSize; i++) {
				refCsfile[i] = pcslist[i].toString(); 
			}
		}
	}else {
		os << "ERROR graspPluginSetting is not found in yaml" << endl;
		cout << "ERROR graspPluginSetting is not found in yaml" << endl;
		targetArmFinger=NULL;
		return false;
	}
	
	
	bodyItemGRC = new cnoid::BodyItem();
	if( bodyItemGRC->loadModelFile(tc->dataFilePath() + "grcTemplateHrp.wrl") ){
		bodyItemGRC->setName("GRC");
		bodyItemRobot()->addSubItem(bodyItemGRC);	/* modified by qtconv.rb 4th rule*/  
	}
	else{
		bodyItemGRC = NULL;
	}
	
	for(int i=0;i<nFing();i++) fingers(i)->coldetLinkPair(targetObject->bodyItemObject);
	arm()->palmObjPair = new ColdetLinkPair(palm(),object() );
	bodyItemRobot()->body()->calcForwardKinematics();
	
	_initrand();
	return true;
}



bool GraspController::doGraspPlanning() {
	
	if ( !targetObject || !targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}
	
	
	int motionN = -1;
	double eval = 1.e10;
	double quality;
	double priority=-1;//reference priority for selecting would be implmented
	
	double grcQuality=0;
	for (int i = 0;i < refSize;i++) {
	cout << "test3c" << endl;
		if ( (grcQuality=returnRefMotion(false, reffile[i])) >=0) {
	cout << "test3a" << endl;
			if ( (quality = getPalmPosture()) < (1.e10 - 1.0)) {
	cout << "test3b" << endl;
//				if (grcQuality*quality < eval || motionN / 3 < i / 3) {
				if (grcQuality*quality < eval ) {
					eval = quality;
					motionN = i;
				}
			}
		}
	}

	cout << "test3" << endl;
	if (motionN < 0) {
		for (int i = 0;i < refCsSize;i++) {
			if (returnRefMotion(true, refCsfile[i])) {
				if ( (quality = getPalmPosture()) < (1.e10 - 1.0)) {
					if (quality < eval) {
						eval = quality;
						motionN = i;
					}
				}
			}
		}
		if (motionN > -1){
			returnRefMotion(true, refCsfile[motionN]);
			os << "cs_mode " ;
			os << refCsfile[motionN] <<" "<< motionN <<" is selected" << endl;
		}
	}else{
		returnRefMotion(false, reffile[motionN]);
		os << reffile[motionN] <<" "<< motionN <<" is selected" << endl;
	}
	cout << "test4" << endl;

	if (motionN < 0) { 
		os << "cannot find palm position" << endl;
		//readMotionFile(refMotion.c_str());
		//bodyItemRobot()->body()->calcForwardKinematics();
		return false ;
	}

	cout << "test5" << endl;
	getPalmPosture();
	

	cout << "test6" << endl;
	arm()->IK_arm(palmPos, palmRot);
	bodyItemRobot()->body()->calcForwardKinematics();
	palmPos = palm()->p;
	palmRot = palm()->R;

	readMotionFile(refMotion.c_str());
	
	cout << "test2" << endl;

//	initialPlan();

	if (sampleFinalPos()){
//		doPickAndPlaceMotionPlanning();
		os << "Success: Grasp Plannng  " << endl;
		return true;
	}
	else{
		os << "Fail: Grasp Plannng" << endl;
		return false;
	}
	
	return true;
	
}


void GraspController::saveGraspPattern(){
	
	tc = PlanBase::instance();
	
	if ( !tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return;
	}
	cout << "test" << endl;
	initial( tc->targetObject, tc->targetArmFinger);
	//this->targetObject = tc->targetObject;
	//this->targetArmFinger = tc->targetArmFinger;
	
	  os << endl;
	  os << "CAUTION:  Please use this command after grasp planning command." << endl;
	  os << endl;
	
	cout << "test" << endl;
	
		targetObject->objVisPos = object()->p;
		targetObject->objVisRot = object()->R;
	
		palmPos = palm()->p;
		palmRot = palm()->R;
	
	  
	  string pfile = tc->dataFilePath() + "preplanning_" + targetObject->bodyItemObject->name() + ".txt";
	  
	  ofstream fout( pfile.c_str(), std::ios::app);

	  while(1){
	  cout << "Rotational direction [x: 0, y: 1, z: 2]: " << endl;
	  cout << endl;
	  int i;
	  cin >> i;

	  Vector3 e(0.0, 0.0, 0.0);
	  e(i)=1.0;
	  
	  cout << "Rotational angle [0-180]: " << endl;
	  cout << endl;
	  double angleStep;
	  cin >> angleStep;
	  angleStep *= 3.1415/180.0;
	
	  if(angleStep==0) angleStep=10000;


	  cout << "Translational direction [x: 0, y: 1, z: 2]: " << endl;
	  cout << endl;
	  cin >> i;

	  Vector3 t(0.0, 0.0, 0.0);
	  t(i)=1.0;

	  cout << "Translation length:" << endl;
	  cout << endl;
	  double tran;
	  cin >> tran;

	  t = tran*t;

	  cout << "Rotate about y axis for 180[deg]? [y/n]" << endl;
	  string yr;
	  cin >> yr;

	  bool y_rot=false;
	  if(yr == "Y" || yr == "y")
		  y_rot = true;


	  for(double angle=0.0; angle<2*3.1415; angle+=angleStep){

		  Matrix3 rot = rodrigues(e, angle);

		  int iter=1;
		  if(y_rot)
		  iter=2;
		  
		  for(int f=0; f<iter; f++){
		  if(f==1)
			  rot = rodrigues(Vector3(0,1,0), 3.141592)*rot;

		  Vector3  tmpPos ((objVisPos()-t) + rot*(palmPos - (objVisPos()-t)));
		  Matrix3 tmpRot (rot*palmRot);

		  Vector3  oPp (trans(objVisRot())*(tmpPos - (objVisPos()-t)));
		  Matrix3 oRp (trans(objVisRot())*tmpRot);
		  
		  fout << 1.0 << " ";
		  
		  for(int i=0;i<3;i++){
			  for(int j=0;j<3;j++){
			  fout << oRp(i,j) << " ";
			  }
		  }
	
		  for(int i=0;i<3;i++){
			  fout << oPp(i) << " ";
		  }

		  for (int i = 1;i < nHandLink();i++){
			  fout <<  handJoint()->link(i)->q <<" ";
		  }
		  
		  fout << endl;
		}

	  }
	  
	  cout << " Continue? [y/n]" << endl;
	  string cnt;
	  cin >> cnt;
	  if(cnt != "Y" && cnt != "y")
		cout << "finish " << endl;
		  break;
	  }

}



// load parameter part of *.prm file
void GraspController ::parseParameters( istream &fp ) {
	char buf[512];
	double v;
	Vector3 w;
	int k;
	const string fingerAlias[5] = {"Thumb_", "Index_", "Middle_", "Ring_", "Little_"};
	
	GetString(fp, buf);
	// skip the first [START] or [BEGIN]
	if (strcmp(buf, "[START]") != 0 && strcmp(buf, "[BEGIN]") != 0 ) {
		BackString(fp, buf);
		return;
	} 

	vector <string> nameFinger(nFing());
	for (int i = 0;i < nFing();i++) {
		stringstream temp;
		temp << "Finger" << i;
		nameFinger[i]   =  (temp).str();
	}
	arm()->palmContact=false;

	while ( GetString(fp, buf) ) {

		for (int i = 0;i < nFing();i++) {
			string temp = nameFinger[i] + "Contact";
			string tempa = fingerAlias[i] + "Contact";
			if ((temp == string(buf)) || (i < 5 && (tempa == string(buf)))) {
				fingers(i)->contact.clear();
				while ( GetLineDouble(fp, v) ) {
					fingers(i)->contact.push_back( (bool)v );
				}
			}
			temp = nameFinger[i] + "Comp";
			tempa = fingerAlias[i] + "Comp";
			if ((temp == string(buf)) || (i < 5 && (tempa == string(buf)))) {
				fingers(i)->compLink.clear();
				while ( GetLineDouble(fp, v) ) {
					fingers(i)->compLink.push_back( (int)v );
				}
			}
			temp = nameFinger[i] + "Close";
			tempa = fingerAlias[i] + "Close";
			if ((temp == string(buf)) || (i < 5 && (tempa == string(buf)))) {
				fingers(i)->close.clear();
				while ( GetLineDouble(fp, v) ) {
					fingers(i)->close.push_back( v );
				}
			}
		}

		if (strcmp(buf, "[END]") == 0) {
			SkipToEOL(fp);
			break;
		}
		else if (strcmp(buf, "palmCloseDir") == 0) {
			arm()->palmContact =true;
			int k=0;
			while ( GetLineDouble(fp, arm()->closeDir[k]) ) k++; 
		}
		else if (strcmp(buf, "Reference_Motion") == 0) {

			GetString(fp, buf);
			refMotion = (string)buf;


		}
		else if (strcmp(buf, "GRCdes_Position") == 0) { //not used parameter

			k = 0;
			while ( GetLineDouble(fp, GRCdes.p[k]) )
				k++;


		} else if (strcmp(buf, "GRCdes_Rpy") == 0) { //not used parameter

			k = 0;
			while ( GetLineDouble(fp, w[k]) )
				k++;

			GRCdes.R = rotFromRpy(w);


		} else if (strcmp(buf, "GRCdes_Edges") == 0) {

			k = 0;
			while ( GetLineDouble(fp, GRCdes.edge[k]) )
				k++;


		}

		else if (strcmp(buf, "GRCmin_Position") == 0) { //not used parameter

			k = 0;
			while ( GetLineDouble(fp, GRCmin.p[k]) ) 
				k++;


		} else if (strcmp(buf, "GRCmin_Rpy") == 0) { //not used parameter

			k = 0;
			while ( GetLineDouble(fp, w[k]) )
				k++;

			GRCmin.R = rotFromRpy(w);


		} else if (strcmp(buf, "GRCmin_Edges") == 0) {

			k = 0;
			while ( GetLineDouble(fp, GRCmin.edge[k]) )
				k++;


		}

		else if (strcmp(buf, "GRCmax_Position") == 0) {

			k = 0;
			while ( GetLineDouble(fp, GRCmax.p[k]) )
				k++;


		} else if (strcmp(buf, "GRCmax_Rpy") == 0) {

			k = 0;
			while ( GetLineDouble(fp, w[k]) )
				k++;

			GRCmax.R = rotFromRpy(w);


		} else if (strcmp(buf, "GRCmax_Edges") == 0) {

			k = 0;
			while ( GetLineDouble(fp, GRCmax.edge[k]) )
				k++;


		} else if (strcmp(buf, "Max_Load") == 0) {

			GetDouble(fp, maxLoad);

		} else if (strcmp(buf, "Min_Load") == 0) {

			GetDouble(fp, minLoad);

		}

		else if (strcmp(buf, "Approach_Vector") == 0) {

			GetDouble(fp, v);
			appVector = (int)v;

		} else if (strcmp(buf, "GRC_Pos_Deviation") == 0) {

			k = 0;
			while ( GetLineDouble(fp, dGRC_Pos_[k]) )
				k++;


		}
	}
	return;
}


void GraspController::doDisplayGRCPosition(){
	
	if(bodyItemGRC){
		Matrix3 gR = bodyItemGRC->body()->link(0)->R; //= palm()->R*(GRCmax.R);
		Vector3 gP = bodyItemGRC->body()->link(0)->p; //= palm()->p+palm()->R*GRCmax_Pos_;
		
		os << "rpy"<< rpyFromRot( Matrix3(trans(palm()->R) * gR)) << endl;
		os << "pos"<<Vector3(trans(palm()->R)* (gP-palm()->p) )<< endl;
	
		
	}
	
}

void GraspController::closeFingers(){
	
	returnRefMotion(false, reffile[0]); //use the fisrt prehension
	bodyItemRobot()->body()->calcForwardKinematics();
	palmPos = palm()->p;
	palmRot = palm()->R;
	readMotionFile(refMotion.c_str());
	
	int cnt=0;
	int nContact=0;
	for(int i=0;i<nFing();i++) for(int j=0;j<fingers(i)->nJoints;j++) if(fingers(i)->contact[j]) nContact++;
	vector <Vector3> objPos(nContact), objN(nContact);
	for(int i=0;i<nFing();i++){
		fingers(i)->contactSearch(cnt, 1000, &objPos[0], &objN[0]);
		fingers(i)->fing_path->joint(0)->q += 3.0*3.14/180.0;
		fingers(i)->fing_path->calcForwardKinematics();
		os << fingers(i)->fing_path->joint(0)->q << endl;
		tc->flush() ;
	}		
	
	
}

bool GraspController::loadAndSelectGraspPattern() {
	PlanBase* pb = PlanBase::instance();

	if ( !pb->targetObject || !pb->targetArmFinger) {
		cout << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	
	double Eval = 1.e10;
	bool found = false;

	double start = getrusage_sec();

	int dim = pb->nHandLink() - 1;

	double qtmp;

	int objtmp;
	Vector3 rpp;
	Matrix3 rpr;

	Matrix3 tmpRot2;
	Vector3 tmpPos2;
	Vector3 palmPos (0, 0, 0);
	Matrix3 palmRot;
	palmRot << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	
//	string pos_data   =   pb->bodyItemRobotPath() + "/data/preplanning_" + pb->targetObject->bodyItemObject->body()->name() + ".txt";
	string pos_data;
	if(pb->targetObject->preplanningFileName.length() )   pos_data   =   pb->dataFilePath() + pb->targetObject->preplanningFileName;
	else pos_data   =   pb->dataFilePath() + "preplanning_" + pb->targetObject->bodyItemObject->name() + ".txt";
 	//os << pos_data << endl;
	ifstream fin_pos(pos_data.c_str());

	string line;

	vector <double> fpos(dim);
	vector<double> tfpos(dim);
	
	pb->setGraspingState(PlanBase::UNDER_GRASPING);


	while (fin_pos) {

		getline(fin_pos, line);
		stringstream li;
		li << line;

		li >> qtmp;

		for (int i = 0;i < 3;i++) {
			for (int j = 0;j < 3;j++) {
				li >> rpr(i, j);
			}
		}
		for (int i = 0;i < 3;i++) {
			li >> rpp(i);
		}

		for (int j = 0;  j < dim; j++) {
			li >> tfpos[j];
		}

		Matrix3 tmpRot( (pb->objVisRot())*rpr );
		Vector3  tmpPos( (pb->objVisRot())*rpp + pb->objVisPos() );

		// --- Evaluation of grasp configuration ---

		bool ikConv = pb->arm()->IK_arm(tmpPos, tmpRot);


//		double  leng = norm2(tmpPos - arm()->arm_path->joint(0)->p);
		double quality = fabs(pb->arm()->arm_path->joint(4)->q) + fabs(pb->arm()->arm_path->joint(5)->q) + fabs(pb->arm()->arm_path->joint(6)->q) ;

		double dist = 0.0;
		double standard[9] = {0,0,0,-0.5,0,1.57,0,0,0};
		for (int i = 0; i < pb->arm()->arm_path->numJoints(); i++){
			double edist =  (pb->arm()->arm_path->joint(i)->q - pb->arm()->armStandardPose[i]); // * 6.28 / (arm_path->joint(i)->ulimit - arm_path->joint(i)->llimit);
			dist +=  edist*edist;
		}
		//for smartpal , will be fixed
		quality = dist;


		if (  (Eval > quality) && ikConv && pb->arm()->checkArmLimit() && !pb->isColliding() ) {

			found = true;

			palmPos = tmpPos;
			palmRot = tmpRot;

			for (int j = 0;  j < dim; j++) {
				fpos[j] = tfpos[j];
			}

			Eval = quality;

			cout << ikConv << " " << quality << endl;

//			oGraspPos = rpp;
//			oGraspRot = rpr;
		}
	}


	if (found) {
		pb->arm()->IK_arm(palmPos, palmRot);
		for (int k = 1;k < pb->nHandLink();k++) pb->handJoint()->link(k)->q = fpos[k-1];
		string HDHpos_data   = pb->dataFilePath() + "grasp_HDH.dat";
		ofstream fHDHpos(HDHpos_data.c_str());
		for (int k = 0;  k < dim; k++) {
			fHDHpos << fpos[k]*180.0 / 3.1415 << " ";
		}
		fHDHpos << 2.0 << endl;
		fHDHpos.close();
		pb->calcForwardKinematics();
		pb->flush();
	} else {
		cout << " No feasible grasping posture found" << endl;
	}
	double end = getrusage_sec();
	cout << "time " << end - start << endl;

	return found;

}
