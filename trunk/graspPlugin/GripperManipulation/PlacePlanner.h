// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef PLACEPLANNER_H
#define PLACEPLANNER_H

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#include <algorithm>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/resource.h>

#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/MessageView>
#include <cnoid/Body>
#include <cnoid/ModelNodeSet>
#include <cnoid/Link>
#include <cnoid/ColdetLinkPair>
#include <cnoid/JointPath>
#include <cnoid/SceneView>
#include <cnoid/SceneBody>
#include <boost/filesystem.hpp>

#include "../Grasp/VectorMath.h"
#include "../Grasp/Arm.h"
#include "../Grasp/PlanBase.h"
#include "ClusterParameter.h"
#include "CollisionPair.h"

namespace grasp{
namespace GripperManipulation{

struct putPosition
{
		std::vector<int> IndexSet;
		int assigned, Index;
};

class PlacePlanner
{

public :
		static PlacePlanner* instance();
		PlacePlanner();
		~PlacePlanner();

		void calcObjPosFaceFace(const cnoid::Vector3& pos, ClusterParameter& env, ClusterParameter& obj, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		void calcPutPos(cnoid::Vector3& pressPos, const cnoid::Matrix3& pressOri, std::vector<cnoid::Vector3>& Po_put, std::vector<cnoid::Matrix3>& Ro_put);
		bool findTargetTriangle( cnoid::Link *targetLink, cnoid::Vector3 &pressPos, cnoid::BodyItemPtr pBody);

		std::ostream& os;
		putPosition putPos;
		CollisionPair *cp;
		ParameterFileData *pf;
		cnoid::Vector3 objPressPos;
		std::string objPressName;
};

}
}

#endif
