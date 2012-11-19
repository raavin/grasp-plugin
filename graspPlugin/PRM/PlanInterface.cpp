// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "PlanInterface.h"

using namespace cnoid;
using namespace grasp;

PlanInterface::PlanInterface(cnoid::BodyItemPtr rob,list<cnoid::BodyItemPtr> env) {
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
	epsilon = 0.005; //default =0.012(larger faster) hrp2demo 0.3
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

PlanInterface::~PlanInterface()
{
	return;
}

bool PlanInterface::call_smoother(vector<VectorXd>& plan)
{
	int qdSize;
	bool map;
	if(plan[0].size() > robItem->body()->numJoints()+5 ){
		qdSize = plan[0].size();
		map = true;
	}
	else{
		qdSize = robItem->body()->numJoints();
		map = false;
	}

	mpkConfig qd(qdSize);
	VectorXd qd_(qdSize);

	mpkRobots robots(robItem);

	vector<mpkConfig> sub_path;
	p2q(plan, sub_path);

#if ADAPT_COLLCHECKER
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0);
#else
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0, epsilon);
#endif
	smoother.smoothe(smoothe_steps);
	smoother.get_path(sub_path);

	plan.clear();
	q2p(sub_path,plan);

	return true;

}

bool PlanInterface::call_planner(vector<VectorXd>& plan_, const vector<int>& SampleDOF)
{

	vector<mpkConfig> plan;

	//double time_start = GetTime();

	mpkRobots robots(robItem);
	int qdSize;
	bool map;
	if(plan_[0].size() > robItem->body()->numJoints()+5 ){
		qdSize = plan_[0].size();
		map = true;
	}
	else{
		qdSize = robItem->body()->numJoints();
		map = false;
	}

	mpkConfig qd(qdSize);
	VectorXd qd_(qdSize);

	p2q(plan_, plan);

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
	q2p(tmp_plan,plan_);

	return true;
	//double delta_t = GetTime() - time_start;
	//cout << delta_t << " s" << endl;
	//planner_time = delta_t;
}

void PlanInterface::p2q(vector<cnoid::VectorXd>& p, vector<mpkConfig>& q){
	int size;
	bool map;
	if(p[0].size() > robItem->body()->numJoints()+5 ){
		size = p[0].size();
		map = true;
	}
	else{
		size = robItem->body()->numJoints();
		map = false;
	}

	mpkConfig qd(size);

	for (unsigned int j=0; j<p.size(); j++ ){

		for(unsigned int i=0; i<robItem->body()->numJoints(); i++){
			qd[i] = (p[j](i) - robItem->body()->joint(i)->llimit)/(robItem->body()->joint(i)->ulimit - robItem->body()->joint(i)->llimit);
		}
		if(map){
			int top = robItem->body()->numJoints();
			for(unsigned int i=0; i<3; i++){
				qd[top+i] = (p[j](top+i) - PlanBase::instance()->llimitMap[i])/(PlanBase::instance()->ulimitMap[i] - PlanBase::instance()->llimitMap[i]);
			}
			top = top+3;
			for(unsigned int i=0; i<3; i++){
				qd[top+i] = (p[j](top+i) - (-7.0))/( 7.0 - (-7.0) );
			}
		}
		q.push_back(qd);
	}

}

void PlanInterface::q2p(vector<mpkConfig>& q, vector<cnoid::VectorXd>& p){
	int size;
	bool map;
	if(p[0].size() > robItem->body()->numJoints()+5 ){
		size = p[0].size();
		map = true;
	}
	else{
		size = robItem->body()->numJoints();
		map = false;
	}
	VectorXd qd_(size);

	for (unsigned int i=0; i<q.size(); i++ ){

		for(unsigned int j=0; j<robItem->body()->numJoints(); j++){
			qd_[j] =  robItem->body()->joint(j)->llimit + q[i][j]*(robItem->body()->joint(j)->ulimit - robItem->body()->joint(j)->llimit);
		}
		if(map){
			int top = robItem->body()->numJoints();
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = PlanBase::instance()->llimitMap[j] + q[i][top+j]*(PlanBase::instance()->ulimitMap[j] - PlanBase::instance()->llimitMap[j]);
			}
			top = top+3;
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = -7.0+ q[i][top+j]*( 7.0 - (-7.0) );
			}
		}
		p.push_back(qd_);
	}
}
