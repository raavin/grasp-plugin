// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "SBLinterface.h"

using namespace cnoid;
using namespace grasp;

SBL::SBL(cnoid::BodyItemPtr rob,list<cnoid::BodyItemPtr> env) {
//: os (MessageView::mainInstance()->cout()){ 

	//arm_path = rob->body()->getJointPath(base, tip);
	nPair = 0;
	for(unsigned int i=0; i<rob->body()->numJoints(); i++)
		for(unsigned int j=i+2; j<rob->body()->numJoints(); j++)
			nPair++;
	//tip_ = tip;
	//base_ = base;
	robItem = rob;
	
	contactPair = new ColdetLinkPairPtr[rob->body()->numJoints() * env.size() + nPair];
	int cnt=0;
	for(unsigned int j=0;j<rob->body()->numJoints();j++){
		list<cnoid::BodyItemPtr>::iterator it = env.begin();
		for(unsigned int k=0;k<env.size();k++){
			contactPair[cnt++] = new ColdetLinkPair(rob->body()->joint(j), (*it)->body()->link(0));
			contact.push_back(false);
			it++;
		}
	}
	
	for(unsigned int i=0; i<rob->body()->numJoints(); i++){
		for(unsigned int j=i+2; j<rob->body()->numJoints(); j++){
			contactPair[cnt++] = new ColdetLinkPair(rob->body()->joint(i), rob->body()->joint(j));
			contact.push_back(false);
		}
	}

	//robots = new mpkRobots(tip, base, rob, env);
	
	delta = 0.01;
	epsilon = 0.05; //default =0.012(larger faster) hrp2demo 0.3
	rho = 0.15;
	max_iter = 30000;
	max_animsteps = 100;
	step_size = 0.1;
	
	database_idx = -1;
	plan_idx = -1;
	planner_time = -999;
	
	smoothe_steps = 50;

	return;
}

SBL::~SBL()
{
	return;
}

bool SBL::call_smoother(vector<VectorXd>& plan)
{

	mpkConfig qd(robItem->body()->numJoints());
	VectorXd qd_(robItem->body()->numJoints());
	mpkRobots robots(robItem);
	
	vector<mpkConfig> sub_path;
	
	for (unsigned int j=0; j<plan.size(); j++ ){
		
		for(unsigned int i=0; i<robItem->body()->numJoints(); i++)
			qd[i] = (plan[j](i) - robItem->body()->joint(i)->llimit)/(robItem->body()->joint(i)->ulimit - robItem->body()->joint(i)->llimit);
		sub_path.push_back(qd);
	}
	
#if ADAPT_COLLCHECKER
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0);
#else
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0, epsilon);
#endif
	smoother.smoothe(smoothe_steps);
	smoother.get_path(sub_path);
	
	plan.clear();
	
	for (unsigned int i=0; i<sub_path.size(); i++ ){
		
		for(unsigned int j=0; j<robItem->body()->numJoints(); j++)
			qd_[j] =  robItem->body()->joint(j)->llimit + sub_path[i][j]*(robItem->body()->joint(j)->ulimit - robItem->body()->joint(j)->llimit);
		plan.push_back(qd_);
	}
	return true;
	
}

bool SBL::call_planner(vector<VectorXd>& plan_, const vector<int>& SampleDOF)
{

	vector<mpkConfig> plan;
	
	//double time_start = GetTime();
	
	mpkRobots robots(robItem);
	
	mpkConfig qd(robItem->body()->numJoints());
	VectorXd qd_(robItem->body()->numJoints());
	
	for(unsigned int j=0; j<plan_.size(); j++){
		
		qd_ = plan_[j];
		
		for(unsigned int i=0; i<robItem->body()->numJoints(); i++)
			qd[i] = (qd_(i) - robItem->body()->joint(i)->llimit)/(robItem->body()->joint(i)->ulimit - robItem->body()->joint(i)->llimit);
		
		plan.push_back(qd);
	}
	
	
	vector<mpkConfig> tmp_plan;
	unsigned int start=0;
	while ( start < plan.size()-1 ) {
		
		unsigned int goal;
		for ( goal = start+1; goal < plan.size(); goal++ )
			;
		if ( goal >= plan.size() ) goal = plan.size()-1;
		
#if ADAPT_COLLCHECKER
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF, epsilon);
#else
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF);
#endif
		list<mpkConfig> clist;
		bool ok = planner.Query(plan[start], plan[goal], clist, rho, max_iter);
		
		if ( !ok ) {
			cerr << "Failure" << endl;
			planner_time = -1;
			return false;
		}
		list<mpkConfig>::iterator it;
		for ( it = clist.begin(); it != clist.end(); ++it )
			tmp_plan.push_back(*it);
		tmp_plan.pop_back();
		
		start = goal;
	}
	tmp_plan.push_back(plan[plan.size()-1]);
	
	plan_.clear();
	for(unsigned int i=0; i<tmp_plan.size(); i++){
		
		for(unsigned int j=0; j<robItem->body()->numJoints(); j++)
			qd_[j] = robItem->body()->joint(j)->llimit + tmp_plan[i][j]*(robItem->body()->joint(j)->ulimit - robItem->body()->joint(j)->llimit);
		
		plan_.push_back(qd_);
	}  
	return true;
	//double delta_t = GetTime() - time_start;
	//cout << delta_t << " s" << endl;
	//planner_time = delta_t;
}
