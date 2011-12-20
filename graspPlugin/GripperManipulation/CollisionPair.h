// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef COLLISIONPAIR_H
#define COLLISIONPAIR_H

#include <cnoid/MessageView>
#include "../Grasp/PlanBase.h"

namespace grasp{
namespace GripperManipulation{

class CollisionPair 
{

public :
		CollisionPair();
		~CollisionPair();

		void setCollisionRob(std::vector<cnoid::ColdetLinkPairPtr>& cPair, ArmPtr arm, FingerPtr fingers[]);
		void setCollisionSelf();
		void setCollisionObj();
		std::ostream& os;

		bool isColliding(std::vector<cnoid::ColdetLinkPairPtr>& cPair);

		std::vector<cnoid::ColdetLinkPairPtr> colPairRight, colPairLeft, colPairSelf, colPairObj;

protected :
};

}
}

#endif
