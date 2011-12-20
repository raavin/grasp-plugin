// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef SBL_INTERFACE_H
#define SBL_INTERFACE_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>
#include <string>
#include <queue>

#include <boost/filesystem.hpp>
//#include <cnoid/Tvmet4d>	/* modified by qtconv.rb 0th rule*/  

#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
//#include <boost/shared_ptr.hpp>
//#include <cnoid/Item>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/YamlNode>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/SignalBuffer>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/FunctionBuffer>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Body>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ModelNodeSet>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Link>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ColdetLinkPair>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  

//#include <cnoid/MatrixSolvers>	/* modified by qtconv.rb 0th rule*/  

#include <cnoid/SceneBody>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/WorldItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SceneBodyManager>	/* modified by qtconv.rb 0th rule*/  

#include <cnoid/EigenTypes>	/* modified by qtconv.rb 0th rule*/  

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

//using namespace std;
//using namespace tvmet;
//using namespace hrp;


namespace grasp{

struct config {
	config() {};
	config(const mpkConfig& q, bool is_key=true)
	{this->q = q; this->is_key = is_key;};
	bool is_key;
	mpkConfig q;
};
	
class SBL
{
	
public :
	static SBL* instance();
	SBL(cnoid::BodyItemPtr rob, list<cnoid::BodyItemPtr> env);
	~SBL();

	
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
