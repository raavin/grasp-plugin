// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef PLAN_INTERFACE_H
#define PLAN_INTERFACE_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>
#include <string>
#include <queue>

#include <boost/filesystem.hpp>
#include <cnoid/MessageView>
#include <cnoid/Body>
#include <cnoid/ModelNodeSet>
#include <cnoid/Link>
#include <cnoid/ColdetLinkPair>
#include <cnoid/JointPath>
#include <cnoid/SceneBody>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/SceneBodyManager>
#include <cnoid/EigenTypes>

#include "mpk_rand.h"
#include "mpk_defs.h"
#include "mpkRobots.h"
#include "sblPlanner.h"
#include "mpkPathSmoother.h"
#include "tspTour.h"
#include "exportdef.h"

//#include "GetTime.h"

#if ADAPT_COLLCHECKER
#define PLANNER "A-SBL"
#else
#define PLANNER "SBL"
#endif

namespace grasp{

struct config {
	config() {};
	config(const mpkConfig& q, bool is_key=true)
	{this->q = q; this->is_key = is_key;};
	bool is_key;
	mpkConfig q;
};

class PlanInterface
{

public :
	static PlanInterface* instance();
	PlanInterface(cnoid::BodyItemPtr rob, list<cnoid::BodyItemPtr> env);
	~PlanInterface();

	vector<bool>contact;
	//cnoid::JointPathPtr arm_path;
	int nPair;

	//std::ostream& os;
	cnoid::ColdetLinkPairPtr *contactPair;

	//cnoid::Link *tip_;
	//cnoid::Link *base_;
	cnoid::BodyItemPtr robItem;
	//cnoid::BodyItemPtr envItem;

	bool call_smoother(vector<cnoid::VectorXd>& plan);
	bool call_planner(vector<cnoid::VectorXd>& plan, const std::vector<int>& trajectoryPlanDOF);

	void p2q(std::vector<cnoid::VectorXd>& p, std::vector<mpkConfig>& q);
	void q2p(std::vector<mpkConfig>& q, std::vector<cnoid::VectorXd>& p);

	double delta;
	double epsilon;
	double rho;
	int max_iter;
	int max_animsteps;
	double step_size;

	string conf_fname;
	vector<config> database;
	int database_idx;
	vector<config> plan;
	int plan_idx;
	double planner_time;

	int smoothe_steps;

	mpkConfig conf_buf;

};


}

#endif
