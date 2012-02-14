// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef CLUSTERPARAMETER_H
#define CLUSTERPARAMETER_H

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/WorldItem>	/* modified by qtconv.rb 0th rule*/  

#include "../Grasp/readtext.h"
#include "../Grasp/PlanBase.h"

namespace grasp{
namespace GripperManipulation{

class ClusterParameter{
 public:
		void deleteCluster();
		void printCluster();

		std::vector<cnoid::Vector3> controlPoints, approachVec, convexhullPoints;
		std::vector<std::vector<cnoid::Vector3> > boundaryPointList;
		cnoid::Vector3 normal, tangent, bbCenter, bbEdge;
		int id;
		std::vector<int> idPair;
		double area;
		int intention;
		bool isPuttingCluster;
};

class ParameterFileData {

public :
		static ParameterFileData* instance();
		void readObjClusterParameters();
		void readEnvClusterParameters();
		void readPRMFile(const char fname[], std::vector<ClusterParameter>& clusterSet);
		void readObjYamlFile(cnoid::BodyItemPtr item, std::vector<ClusterParameter>& clusterSet);
		void readEnvYamlFile(cnoid::BodyItemPtr item, std::vector<ClusterParameter>& clusterSet);
		void calcIDPair(int intention);
		std::vector<std::vector<int> > idPairs;
		std::vector<ClusterParameter> objClusters, handClusters, envClusters, objEnvClusters;
		std::vector<double> gainParameter, motionTime;
		std::vector<int> graspPostureCandidate;
protected :
		std::string bodyItemObjectPath;
};

}
}

#endif