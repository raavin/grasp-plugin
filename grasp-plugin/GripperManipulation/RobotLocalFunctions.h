// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef ROBOTLOCALFUNCTIONS_H
#define ROBOTLOCALFUNCTIONS_H

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>

#include "../Grasp/VectorMath.h"
#include "../Grasp/PlanBase.h"

namespace grasp{
namespace GripperManipulation{

class RobotLocalFunctions
{

public :
		void calcPalmFingParam(int robot, const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Vector3& pNcr1, const cnoid::Vector3& pTcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta);

		void Calibration(int robot);
		void writeFile(int robot, const std::vector<cnoid::VectorXd>& jointSeq, std::vector<double>& motionTimeSeq, bool hasOffset=true);
		enum Robots {PA10, HIRO, HRP2};

//private:
		void writeJointSeq(char * jointlogfile, const unsigned int filelength, const std::vector<cnoid::VectorXd>& jointSeq, std::vector<double>& motionTimeSeq);
		void convertAngles(const cnoid::VectorXd & seq, std::vector<double> & angles, double off);
		void checkAngles(std::vector<double> & angles);

};

}
}

#endif
