/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _FINGER_H
#define _FINGER_H

#include <stdlib.h>
#include <time.h>
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  

#include "exportdef.h"

namespace grasp{


class Finger;
typedef Finger* FingerPtr;

class Finger{
	public:
		Finger(cnoid::BodyPtr body, cnoid::Link *palm, cnoid::Link *tip);
		~Finger() {};
		int number;
		std::vector<bool>contact;
		std::vector <int>compLink;
		std::vector <double>close;
		cnoid::JointPathPtr fing_path;
		cnoid::Link *tip;
		int nJoints;

		std::vector<double> fingerOpenPose, fingerGraspPose;
	
		void coldetLinkPair(cnoid::BodyItemPtr bo);
		
		//virtual bool IK_fingContact_Envelope(int lk, int& iter, const cnoid::Vector3 &p);
		virtual bool closeFinger(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN);
		virtual int contactSearch(int&cnt, int iter, cnoid::Vector3* oPos, cnoid::Vector3* objN);
		virtual bool fingtipGrasp(void);
		virtual bool checkJointLimit(cnoid::Link* joint);
		virtual bool checkFingLimit();
		cnoid::ColdetLinkPairPtr *linkObjPair;
			
		cnoid::Link* joint(int i){
			return fing_path->joint(i);	
		}
//		std::ostream& os;
};

}

#endif
