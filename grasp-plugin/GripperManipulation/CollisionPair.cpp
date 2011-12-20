// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */
#include <iostream>
#include "CollisionPair.h"

#define m_pi 3.141592
#define minDistance 0.001

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

CollisionPair::CollisionPair()  : 	os (MessageView::mainInstance()->cout() ) {
}

CollisionPair::~CollisionPair() {
}


void CollisionPair::setCollisionRob(std::vector<cnoid::ColdetLinkPairPtr>& cPair, ArmPtr arm, FingerPtr fingers[])
{
		int nj = arm->arm_path->numJoints()-1;

		int numContacts = nj*PlanBase::instance()->bodyItemEnv.size() + 1 + PlanBase::instance()->bodyItemEnv.size()*PlanBase::instance()->targetArmFinger->nFing*fingers[0]->nJoints;
		
		cPair.resize(numContacts);

		int i=0;
		list<cnoid::BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin();
		for(int j=0; j<nj*PlanBase::instance()->bodyItemEnv.size(); j++){
				cPair[i++] = new ColdetLinkPair(arm->arm_path->joint(j%nj+1), (*it)->body()->link(0));
				if( (j%nj)==(nj-1) ) it++;
		}
		cPair[i++] = new ColdetLinkPair(arm->arm_path->joint(nj), PlanBase::instance()->targetObject->object );
		
		for(list<cnoid::BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it != PlanBase::instance()->bodyItemEnv.end(); ++it)
				for(int j=0; j<PlanBase::instance()->targetArmFinger->nFing; j++)
						for(int k=0; k<fingers[j]->nJoints; k++)
								cPair[i++] = new ColdetLinkPair(fingers[j]->fing_path->joint(k), (*it)->body()->link(0));

}

void CollisionPair::setCollisionSelf()
{
		int i=0;
		for(int j=4; j<PlanBase::instance()->bodyItemRobot()->body()->numLinks(); j++)
				for(int k=j+2; k<PlanBase::instance()->bodyItemRobot()->body()->numLinks(); k++)
						i++;
		
		colPairSelf.resize(i);
		
		i=0;
		int l=0;
		for(int j=4; j<PlanBase::instance()->bodyItemRobot()->body()->numLinks(); j++)
				for(int k=j+2; k<PlanBase::instance()->bodyItemRobot()->body()->numLinks(); k++)
						colPairSelf[i++] = new ColdetLinkPair(PlanBase::instance()->bodyItemRobot()->body()->link(j), PlanBase::instance()->bodyItemRobot()->body()->link(k));
}


void CollisionPair::setCollisionObj()
{
		int numContacts = PlanBase::instance()->bodyItemEnv.size();
		
		colPairObj.resize(numContacts);
		
		int i=0;
		for(list<cnoid::BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it != PlanBase::instance()->bodyItemEnv.end(); ++it)
				colPairObj[i++] = new ColdetLinkPair(PlanBase::instance()->targetObject->object, (*it)->body()->link(0));
		
		
}

bool CollisionPair::isColliding(std::vector<cnoid::ColdetLinkPairPtr>& cPair)
{

		  for(unsigned int l=0; l<cPair.size(); l++){
				cPair[l]->model(0)->setPosition(cPair[l]->link(0)->R, cPair[l]->link(0)->p);
				cPair[l]->model(1)->setPosition(cPair[l]->link(1)->R, cPair[l]->link(1)->p);
		  }												
		  
		  double p1[3] = {0}, p2[3] = {0};
		  int tid1, tid2;
		  
		  for(int l=cPair.size()-1; l>-1; l--)
				  if( cPair[l]->computeDistance(tid1, &p1[0], tid2, &p2[0]) <= 0.0001 )
						  return true;
		  
		  return false;
}
