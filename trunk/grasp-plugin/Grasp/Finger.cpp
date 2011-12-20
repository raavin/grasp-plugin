// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "Finger.h"

#include <cnoid/JointPath>
#include <iostream>  

#include "VectorMath.h"
#include "GraspController.h"

//#define DEBUG_MODE

using namespace std;
using namespace cnoid;
using namespace grasp;

Finger::Finger(BodyPtr body, Link *palm, Link *tip)
{  
	fing_path = body->getJointPath(palm, tip);
	nJoints = fing_path->numJoints();
	for(int j=0;j<nJoints;j++){
		compLink.push_back(0);
	}
	for(int j=0;j<nJoints;j++){
		contact.push_back(false);
		close.push_back(0.002);
	}
	contact.push_back(true);
	
	return ;	
}

void Finger::coldetLinkPair(cnoid::BodyItemPtr bo){
	linkObjPair = new ColdetLinkPairPtr[nJoints];
	for(int j=0;j<nJoints;j++){
		linkObjPair[j] = new ColdetLinkPair(fing_path->joint(j), bo->body()->link(0));
	}
}


bool Finger::fingtipGrasp(void) {
	int s=0;
	for(int i=0;i<nJoints;i++) if(contact[i]) s++;
	if (s > 1 ) return false;
	return true;
}


bool Finger::closeFinger(int lk, int iter, Vector3 &oPos, Vector3 &objN) {
	
	double dsn_old = 100.0, sgn = 1.0;
	//double distance = norm2(fing_path->joint(lk)->p-fing_path->joint(compLink[lk])->p);
	
	double epsiron = close[lk]; //0;
	//if(distance ==0) epsiron = close[lk];
	//else epsiron = 0.002/distance;

	double delta = 0.0;

	bool finish = false;
	for (int loop = 0; loop < iter; loop++) {
		

		fing_path->joint(compLink[lk])->q += delta;

		if(PlanBase::instance()->interLinkList.size()>0)
			PlanBase::instance()->setInterLink();

		if (! checkJointLimit(fing_path->joint(compLink[lk])) )  {
			iter = 0;
		}
		
		fing_path->calcForwardKinematics();
#ifdef DEBUG_MODE
		PlanBase::instance()->flush();
#endif
		Vector3 Po, Pf;
		double dsn1=1;
		if( (lk-1) >= 0) dsn1=PlanBase::calcContactPoint(linkObjPair[lk-1], Po, Pf, objN); 
		double dsn  = PlanBase::calcContactPoint(linkObjPair[lk], Po, Pf, objN);
		

		if ( ((dsn > 0 && dsn1 > 0) && fingtipGrasp()) || (dsn>0 && !fingtipGrasp()) ){

			oPos = Po;

			//Close direction is fixed now, since dead lock sometimes happens  
			// if (fabs(dsn - dsn_old) != 0.0) sgn = -(dsn - dsn_old) / fabs(dsn - dsn_old);

			delta = sgn*epsiron;

			if (finish && dsn < 0.003) return true;
			else if (finish && dsn >= 0.003) return false;

		} else {
			delta = - sgn*epsiron*0.4;

			finish = true;
		}
		//cout <<"test" <<number <<" " <<delta << " "<< dsn << " "<< dsn_old << endl;

		dsn_old = dsn;
	}

	return false;
}

int Finger::contactSearch(int& cnt,  int iter, Vector3* oPos, Vector3* objN){ 

	for(unsigned int i=0; i<contact.size(); i++){
		if(contact[i]){
			if (closeFinger(i, iter, oPos[cnt], objN[cnt])) 
				cnt++;
		}
	}
}


bool Finger::checkJointLimit(Link* joint){

	if (joint->q < joint->llimit ) {
		joint->q = joint->llimit;
		return false;
	}
	if (joint->q > joint->ulimit) {
		joint->q = joint->ulimit;
		return false;
	}
	return true;
}

bool Finger::checkFingLimit(){

	bool withinLimit = true;
	for (int i = 0; i < fing_path->numJoints(); i++)
		if (fing_path->joint(i)->ulimit < fing_path->joint(i)->q  ||  fing_path->joint(i)->llimit > fing_path->joint(i)->q)
			withinLimit = false;

	return withinLimit;
}
