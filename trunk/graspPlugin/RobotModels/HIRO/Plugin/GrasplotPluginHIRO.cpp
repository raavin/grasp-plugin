// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include <dlfcn.h>
#include "GrasplotPluginHIRO.h"
#include "OpenRAVE/OpenRAVE_IK.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

bool HIRO_Arm::IK_arm(const Vector3& p, const Matrix3& R){
		double phi=0;
		VectorXd q_old(nJoints);
		for(int i=0;i<nJoints;i++){
				q_old[i] = armStandardPose[i];
		}
		return IK_arm( p,  R, phi,  q_old);
}

bool  HIRO_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){

		if(base->name() != "WAIST"){
				cout << "Set base as WAIST in HIRO.yaml. Exiting.." << endl;
				return false;
		}

		int n = arm_path->numJoints();
		Matrix33 R = Rp*arm_path->joint(n-1)->Rs; //R -> attitude()

		
		//for ver4.6
		//Coord trans from Choreonoid to OpenRAVE
		/*
		Vector3 dBasePos, dBaseRpy;
		if(arm_path->joint(n-1)->name()=="RARM_JOINT5"){
				dBasePos = (Vector3(0.324904, -0.140932, -0.0119343)-Vector3(0.324897, -0.138604, -0.0206276)); 
				dBaseRpy = (Vector3(2.87979, -0.000769192, -3.14139)-Vector3(1.56782,   -1.309,   -1.56772)); 
		}
		else{
				dBasePos = (Vector3(0.324904, 0.140932, -0.0119343)-Vector3(0.324897, 0.138604, -0.0206276)); 
				dBaseRpy = (Vector3(-2.87979, -0.000769192, 3.14139)-Vector3(-1.56782,   -1.309,   1.56772)); 
		}

		Vector3 wristOffset(-0.059, 0,0); //Force sensor offset

		Vector3 bp = base->attitude().transpose()*(p - R*wristOffset - base->p) + dBasePos;
		Matrix3 bR = rotFromRpy(rpyFromRot(base->attitude().transpose()*R) + dBaseRpy);
		*/

		//for ver4.1

		Vector3 wristOffset(-0.059, 0,0); //Force sensor offset
		if(arm_path->joint(n-1)->name()=="RARM_JOINT5")
				wristOffset = Vector3(-0.009,0,0); //Maybe a bug in IKfast

		Vector3 bp = base->attitude().transpose()*(p - R*wristOffset - base->p);
		Matrix3 bR = base->attitude().transpose()*R;

		void *ik_handle;
		bool (*ik_)(const IKReal*, const IKReal*, const IKReal*,std::vector<IKSolution>&);

		if(arm_path->joint(n-1)->name()=="RARM_JOINT5"){

				ik_handle = dlopen("extplugin/graspPlugin/RobotModels/HIRO/Plugin/ikfast.HIRONX_RARM.i686.so", RTLD_LAZY);

				arm_path->joint(0)->q = atan2(bp(1), bp(0)) + asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));
		}
		else if(arm_path->joint(n-1)->name()=="LARM_JOINT5"){

				ik_handle = dlopen("extplugin/graspPlugin/RobotModels/HIRO/Plugin/ikfast.HIRONX_LARM.i686.so", RTLD_LAZY);

				arm_path->joint(0)->q = atan2(bp(1), bp(0)) - asin(0.145/sqrt(bp(0)*bp(0)+bp(1)*bp(1)));
		}

		if(fabs(phi)>0.0001) arm_path->joint(0)->q = phi;

		if(!ik_handle) cout << "loading error" << endl;

		Matrix3 Rz = rotFromRpy(0,0,arm_path->joint(0)->q);
		
		Vector3 p0 = Rz.transpose()*bp;
		Matrix3 R0 = Rz.transpose()*bR;

		IKReal p_[3],R_[9];
		for(int i=0; i<3; i++){
				p_[i] = p0(i);
				for(int j=0; j<3; j++)
						R_[3*i+j] = R0(i,j);
		}
		
		ik_ = (bool (*)(const IKReal*, const IKReal*, const IKReal*, std::vector<IKSolution>&))dlsym(ik_handle, "ik");

		bool analytic = true;
		bool solved = true;

		vector<IKSolution> vsolutions;
		if(! ik_(p_, R_, NULL, vsolutions) ) solved = false;

		if(solved){
				vector<IKReal> sol(6);
				bool ret = false;
				for(std::size_t i = 0; i < vsolutions.size(); ++i){
						vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
						vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);
						for( std::size_t j = 0; j < sol.size(); ++j)
								arm_path->joint(j+1)->q = sol[j];
						if(Arm::checkArmLimit()){
								ret = true;
								break;
						}
				}

				if(!ret) solved = false;
		}
		
		//if(solved) cout << "IK fast solved " << endl;

		if(!solved) solved = Arm::IK_arm(p, Rp);

		//if(solved) cout << "IKfast not solvable. Used numerical solution instead." << endl;

		if(!solved) return false;

		while(arm_path->joint(6)->q < arm_path->joint(6)->llimit ) arm_path->joint(6)->q += 2*m_pi;
		while(arm_path->joint(6)->q > arm_path->joint(6)->ulimit ) arm_path->joint(6)->q -= 2*m_pi;

		while( arm_path->joint(6)->q - q_old(6) >  m_pi && arm_path->joint(6)->q-2*m_pi > arm_path->joint(6)->llimit)   arm_path->joint(6)->q -= 2*m_pi;
		while( arm_path->joint(6)->q - q_old(6) < -m_pi && arm_path->joint(6)->q+2*m_pi < arm_path->joint(6)->ulimit)   arm_path->joint(6)->q += 2*m_pi;

		if(!Arm::checkArmLimit()) return false;

		arm_path->calcForwardKinematics();

		return true;
}

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new HIRO_Arm(body, base, palm);      
}
