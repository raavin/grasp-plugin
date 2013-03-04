// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "TrajectoryPlanner.h"

#include<stdio.h>

#include <cnoid/LinkPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/TimeBar>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/

#ifndef WIN32
#include <dirent.h>
#endif

#include "PlanInterface.h"

#include "../Grasp/GraspController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

/*
bool PoseSeqItemGrasp::interpolationGrasp(){
	return false;
}
*/

bool  TrajectoryPlanner::updateTrajectoryFromMotion(const BodyMotionPtr motionObject, const BodyMotionPtr motionRobot, vector<MotionState>& motionSeq){

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr objectBody = gc->targetObject->bodyItemObject->body();
		const int numJoints = objectBody->numJoints();
		const int numLinksToPut = objectBody->numLinks() ;
		motionObject->setDimension(numFrames, numJoints, numLinksToPut);
		motionObject->setFrameRate(frameRate);

#ifdef CNOID_10_11
		MultiAffine3Seq& pseq = *motionObject->linkPosSeq();
#else
		MultiSE3Seq& pseq = *motionObject->linkPosSeq();
#endif
		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
		BodyPtr robotBody = gc->bodyItemRobot()->body();

		const int numJointsRobot = robotBody->numJoints();

		Link* rootLink = objectBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p;
		//orgp.R = rootLink->R;

		vector<MotionState>::iterator itg = motionSeq.begin();
		gc->setGraspingState( itg->graspingState );
		gc->setGraspingState2( itg->graspingState2 );
		itg++;

		gc->object()->p = gc->objVisPos();
		gc->object()->R = gc->objVisRot();

		for(int frame = 0; frame < numFrames; ++frame){
			if( (itg != motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
				gc->setGraspingState( itg->graspingState );
				gc->setGraspingState2( itg->graspingState2 );
				itg++;
			}
#ifdef CNOID_10_11
			MultiValueSeq::View qs = qseqRobot.frame(frame);
#else
			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
#endif
			for(int i=0; i < numJointsRobot; ++i){
				robotBody->joint(i)->q = qs[i];
			}
			gc->calcForwardKinematics();
#ifdef CNOID_10_11
			Affine3& p = pseq.at(frame, 0);
			p.translation() = gc->object()->p;
			p.linear() = gc->object()->R;
#else
			SE3& p = pseq.at(frame, 0);
			p.set(gc->object()->p, gc->object()->R);
#endif
		}

		// store the moved state
		rootLink->p = gc->object()->p;
		rootLink->R = gc->object()->R;
		objectBody->calcForwardKinematics();
		gc->flush();

		return true;
}


bool  TrajectoryPlanner::outputTrajectoryForOpenHRP(const BodyMotionPtr motionRobot){
	BodyMotionPtr motionObject;
	vector<MotionState> motionSeq;

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr robotBody = gc->body();
		const int numJoints = robotBody->numJoints();
		const int numLinksToPut = robotBody->numLinks() ;

#ifdef CNOID_10_11
		MultiAffine3Seq& pseq = *motionRobot->linkPosSeq();
#else
		MultiSE3Seq& pseq = *motionRobot->linkPosSeq();
#endif
		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
//		BodyPtr robotBody = gc->body();

		const int numJointsRobot = robotBody->numJoints();

		Link* rootLink = robotBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p;
		//orgp.R = rootLink->R;
		Link* rfoot = robotBody->link("RLEG_JOINT5");
		Vector3 rfootp= rfoot->p;
		Matrix3 rfootR= rfoot->R;


		gc->object()->p = gc->objVisPos();
		gc->object()->R = gc->objVisRot();

		ofstream pfile ( "grasp.pos") ;
		ofstream zfile ( "grasp.zmp") ;
		ofstream rfile ( "grasp.rpy") ;

		for(int frame = 0; frame < numFrames; ++frame){
#ifdef CNOID_10_11
			MultiValueSeq::View qs = qseqRobot.frame(frame);
#else
			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
#endif
			for(int i=0; i < numJointsRobot; ++i){
				robotBody->joint(i)->q = qs[i];
			}
			robotBody->calcForwardKinematics();
			Vector3 waistp = rfootR*rfoot->R.transpose()*(rootLink->p - rfoot->p)+rfootp;
			Matrix3 waistR = rfootR*rfoot->R.transpose()*rootLink->R;

#ifdef CNOID_10_11
			Affine3& p = pseq.at(frame, 0);
			p.translation() = gc->object()->p;
			p.linear() = gc->object()->R;
#else
			SE3& p = pseq.at(frame, 0);
			p.set(waistp, waistR);
#endif

			Vector3 wporg = rootLink->p;
			Matrix3 wRorg = rootLink->R;

			rootLink->p = waistp;
			rootLink->R = waistR;
			robotBody->calcForwardKinematics();
			double time = (double)frame/frameRate;

			Vector3 cm = robotBody->calcCM();
			cm[2]=0;
			Vector3 zmp = (rootLink->R.transpose()*(cm-rootLink->p));
			Vector3 rpy = (rpyFromRot(rootLink->R));

			pfile << time ;
			for(int i=0; i < numJointsRobot; ++i){
				pfile << " " <<robotBody->joint(i)->q*180.0/M_PI;
			}
			pfile << endl;

			zfile << time << " " << zmp[0] << " " << zmp[1] << " " << zmp[2] << endl;
			rfile << time << " " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;

			rootLink->p = wporg;
			rootLink->R = wRorg;
		}
		return true;
}

TrajectoryPlanner::TrajectoryPlanner(int id){

		PlanBase* gc = PlanBase::instance();
		static int id_ =0;
		if(id==0){
			id = id_;
			id_ ++;
		}
		stringstream name ;
		name  << "GraspPoseSeqItem" << id;

		poseSeqItemRobot = new PoseSeqItem();
		poseSeqItemRobot->setName(name.str());
		gc->bodyItemRobot()->addSubItem(poseSeqItemRobot);	/* modified by qtconv.rb 4th rule*/

		if(gc->targetObject){
				poseSeqItemObject = new PoseSeqItem();
				poseSeqItemObject->setName(name.str());
				gc->targetObject->bodyItemObject->addSubItem(poseSeqItemObject);	/* modified by qtconv.rb 4th rule*/
		}

}

void TrajectoryPlanner::setStartPos() {
		PlanBase* gc = PlanBase::instance();
}

bool TrajectoryPlanner::doTrajectoryPlanning() {
		ItemTreeView::mainInstance()->clearSelection();
		PlanBase* gc = PlanBase::instance();

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		MotionState tempState = gc->getMotionState();
		gc->initialCollision();

		vector<MotionState> inputMotionSeq;
		bool outputGraspMotionSeq = true;

		if(gc->jointSeq.size()>1){
			for(unsigned int i=0; i<gc->jointSeq.size(); i++){
				gc->setGraspingState(gc->graspingStateSeq[i]);
				gc->setGraspingState2(gc->graspingStateSeq2[i]);
				if(gc->objectContactStateSeq.size()>0) gc->setObjectContactState(gc->objectContactStateSeq[i]);
				if(gc->pathPlanDOFSeq.size()>0) gc->pathPlanDOF = gc->pathPlanDOFSeq[i];
				MotionState temp = gc->getMotionState();
				temp.jointSeq = gc->jointSeq[i];
				inputMotionSeq.push_back(temp);
			}
		}
		else if ( gc->graspMotionSeq.size() > 1){
			inputMotionSeq = gc->graspMotionSeq;
		}
		else {
			outputGraspMotionSeq = false;
			//gc->graspMotionSeq.clear();
			MotionState startMotionState, endMotionState;

			if(gc->startMotionState.id > 0){
				startMotionState = gc->startMotionState;
			}
			else{
				MotionState temp = gc->getMotionState();
				for(int i=0;i<temp.jointSeq.size();i++) temp.jointSeq[i]=0;
				temp.id = 1;
				startMotionState = temp;
			}
			if(gc->endMotionState.id > 0){
				endMotionState = gc->endMotionState;
			}
			else{
				MotionState temp = gc->getMotionState();
				temp.id = 2;
				endMotionState = temp;
			}

			if( (startMotionState.pos - endMotionState.pos).norm() > 1.e-10){
				gc->setTrajectoryPlanMapDOF();
			}else{
				gc->setTrajectoryPlanDOF();
			}
			startMotionState.pathPlanDOF = gc->pathPlanDOF;
			endMotionState.pathPlanDOF = gc->pathPlanDOF;
			inputMotionSeq.push_back(startMotionState);
			inputMotionSeq.push_back(endMotionState);
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);

		grasp::PlanInterface planner(gc->bodyItemRobot(), gc->bodyItemEnv);

		vector<VectorXd> config, config_tmp;

		bool successAll=true;

/*		if(gc->jointSeq.size()>1){
			useGraspMotionSeq = false;
			gc->graspMotionSeq.clear();
			for(unsigned int i=0; i<gc->jointSeq.size()-1; i++){
				config_tmp.clear();

				gc->setGraspingState(gc->graspingStateSeq[i]);
				gc->setGraspingState2(gc->graspingStateSeq2[i]);
				if(gc->objectContactStateSeq.size()>0)
						gc->setObjectContactState(gc->objectContactStateSeq[i]);
				if(gc->pathPlanDOFSeq.size()>0)
						gc->pathPlanDOF = gc->pathPlanDOFSeq[i];

				VectorXd cfull(gc->jointSeq[i].size()+6);
				cfull << gc->jointSeq[i], gc->body()->link(0)->p, rpyFromRot(gc->body()->link(0)->R);
				config_tmp.push_back(cfull);

				cfull << gc->jointSeq[i+1], gc->body()->link(0)->p, rpyFromRot(gc->body()->link(0)->R);
				config_tmp.push_back(cfull);

				bool success = planner.call_planner(config_tmp, gc->pathPlanDOF);
				if(success) planner.call_smoother(config_tmp);
				else        successAll = false;

				for(unsigned int j=0; j<config_tmp.size(); j++){
					int k=i, l=i+1;
					if(j==0)                   l=i;
					if(j==config_tmp.size()-1) k=i+1;

					motionSeq.push_back( MotionState( config_tmp[j], gc->graspingStateSeq[k], gc->graspingStateSeq2[k]) );

					if(gc->motionTimeSeq.size()>0)
								(motionSeq.back()).motionTime = gc->motionTimeSeq[l];

					if(j<config_tmp.size()-1 || i==gc->jointSeq.size()-2)
							gc->graspMotionSeq.push_back(motionSeq.back());
				}

			}
		}else */
		if(inputMotionSeq.size() > 1){
			gc->graspMotionSeq.clear();
			for(unsigned int i=0; i<inputMotionSeq.size()-1; i++){
				config_tmp.clear();

				gc->setMotionState(inputMotionSeq[i]);
				//if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
				VectorXd cfull(inputMotionSeq[i].jointSeq.size()+6);
				cfull << inputMotionSeq[i].jointSeq, inputMotionSeq[i].pos, inputMotionSeq[i].rpy;
				config_tmp.push_back(cfull);

				cfull << inputMotionSeq[i+1].jointSeq, inputMotionSeq[i+1].pos, inputMotionSeq[i+1].rpy;
				config_tmp.push_back(cfull);

				bool success = planner.call_planner(config_tmp, inputMotionSeq[i].pathPlanDOF);
				if(!success) successAll=false;

				planner.call_smoother(config_tmp);

				for(unsigned int j=0; j<config_tmp.size(); j++){
					int l=i+1;
					if(j==0) l=i;

					gc->body()->link(0)->p = config_tmp[j].segment<3>(gc->body()->numJoints());
					gc->body()->link(0)->R = rotFromRpy( config_tmp[j].segment<3>(gc->body()->numJoints()+3) );
					for(int k=0;k<gc->bodyItemRobot()->body()->numJoints();k++) gc->bodyItemRobot()->body()->joint(k)->q = config_tmp[j][k];
					gc->setInterLink();
					MotionState temp = gc->getMotionState();
					motionSeq.push_back( temp );

					if(outputGraspMotionSeq){
						if( l < gc->motionTimeSeq.size()  )  temp.motionTime = gc->motionTimeSeq[l];
						if(j<config_tmp.size()-1 || i==inputMotionSeq.size()-2){
							gc->graspMotionSeq.push_back(temp);
						}
					}
				}
			}
		}
		else{
			return false;
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		if(gc->targetObject){
			gc->object()->p = gc->objVisPos();
			gc->object()->R = gc->objVisRot();
		}

		gc->setMotionState(tempState);
		gc->bodyItemRobot()->body()->calcForwardKinematics();


		PosePtr pose = new Pose(gc->bodyItemRobot()->body()->numJoints());
		for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
			pose->setJointPosition(gc->bodyItemRobot()->body()->joint(i)->jointId, gc->bodyItemRobot()->body()->joint(i)->q);
		}

		PosePtr poseObject = new Pose(1);
		if(gc->targetObject){
			poseObject->setBaseLink(0, gc->objVisPos(), gc->objVisRot());
			poseSeqItemObject->poseSeq()->insert(poseSeqItemObject->poseSeq()->end(), 0 , poseObject);
		}

		vector<VectorXd>::iterator it = config.begin();
		double time = 0;
		for(int j=0;j<motionSeq.size();j++){
			PosePtr pose_ = new Pose(*pose);
			pose_->setBaseLink(0, motionSeq[j].pos, rotFromRpy(motionSeq[j].rpy) );
			for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
				pose_->setJointPosition(i, motionSeq[j].jointSeq[i]);
			}
			motionSeq[j].time = time;
			poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), time , pose_);
			time  += 1.0;
		}
		poseSeqItemRobot->updateInterpolation();
		poseSeqItemRobot->updateTrajectory();
		ItemTreeView::mainInstance()->selectItem( poseSeqItemRobot->bodyMotionItem() );

		//outputTrajectoryForOpenHRP(poseSeqItemRobot->bodyMotionItem()->motion());

		if(gc->targetObject){
			gc->object()->p = gc->objVisPos();
			gc->object()->R = gc->objVisRot();
			poseSeqItemObject->updateInterpolation();
			updateTrajectoryFromMotion(poseSeqItemObject->bodyMotionItem()->motion(), poseSeqItemRobot->bodyMotionItem()->motion(),motionSeq);
			poseSeqItemObject->bodyMotionItem()->notifyUpdate();
			ItemTreeView::mainInstance()->selectItem( poseSeqItemObject->bodyMotionItem() );
		}
/*
		DIR *pDir;
		pDir = opendir("extplugin/graspPlugin/RobotInterface");

		if(pDir != NULL){

				ofstream gout("extplugin/graspPlugin/RobotInterface/data/grasp.mat");

				double p = p=180.0/3.1415;
				gout <<"6 7";
				for(unsigned int i=1; i<motionSeq.size(); i++){
						gout << endl;
						for(int j=0;j<gc->bodyItemRobot()->body()->numJoints();j++)
								gout << motionSeq[i].jointSeq[j]*p << " ";
						gout << motionSeq[i].graspingState;

				}
		}
		*/

		return successAll;
}





