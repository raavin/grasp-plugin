// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "mpkRobots.h"

using namespace cnoid;

mpkRobots::mpkRobots( cnoid::BodyItemPtr robots_)
{
       robots = robots_;

       //arm_path = robots->body()->getJointPath(base, tip);
       nJoints = robots->body()->numJoints();

       param_opts = new param_info[nJoints];

	return ;	
}

void mpkRobots::set_config(mpkConfig &q)
{

	for(int i=0; i<nJoints; i++)
		robots->body()->joint(i)->q = robots->body()->joint(i)->llimit + (robots->body()->joint(i)->ulimit - robots->body()->joint(i)->llimit)*q[i];

  return;
}

void mpkRobots::get_config(mpkConfig &q)
{

	for(int i=0; i<nJoints; i++)
		q[i] = (robots->body()->joint(i)->q - robots->body()->joint(i)->llimit)/(robots->body()->joint(i)->ulimit - robots->body()->joint(i)->llimit);

  return;
}

void mpkRobots::compute_forward_kinematics()
{
  robots->body()->calcForwardKinematics();

  return;
}

