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

#include <dirent.h>

#include "SBLinterface.h"

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

		//std::vector<MotionState> motionSeq_tmp;

		gc->initialCollision();
		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);

		iniJoint.resize(gc->bodyItemRobot()->body()->numJoints());
		for(int i=0; i<gc->bodyItemRobot()->body()->numJoints(); i++)
				iniJoint(i) = 0;

		finJoint.resize(gc->bodyItemRobot()->body()->numJoints());
		for(int i=0; i<gc->bodyItemRobot()->body()->numJoints(); i++)
				finJoint(i) = gc->bodyItemRobot()->body()->joint(i)->q;


		grasp::SBL planner(gc->bodyItemRobot(), gc->bodyItemEnv);

		vector<VectorXd> config, config_tmp;

		bool successAll=true;

		if(gc->jointSeq.size()>1){
				gc->graspMotionSeq.clear();

				for(unsigned int i=0; i<gc->jointSeq.size()-1; i++){
						config_tmp.clear();

						gc->setGraspingState(gc->graspingStateSeq[i]);
						gc->setGraspingState2(gc->graspingStateSeq2[i]);
						if(gc->objectContactStateSeq.size()>0)
								gc->setObjectContactState(gc->objectContactStateSeq[i]);
						if(gc->pathPlanDOFSeq.size()>0)
								gc->pathPlanDOF = gc->pathPlanDOFSeq[i];

						config_tmp.push_back(gc->jointSeq[i]);
						config_tmp.push_back(gc->jointSeq[i+1]);

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
		}else if(gc->graspMotionSeq.size() > 1){

				for(unsigned int i=0; i<gc->graspMotionSeq.size()-1; i++){
						config_tmp.clear();

						gc->setGraspingState(gc->graspMotionSeq[i].graspingState);
						gc->setGraspingState2(gc->graspMotionSeq[i].graspingState2);
						gc->setObjectContactState(gc->graspMotionSeq[i].objectContactState);
						gc->pathPlanDOF = gc->graspMotionSeq[i].pathPlanDOF;
						if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
//						if(gc->pathPlanDOFSeq.size()>0)
	//							gc->pathPlanDOF = gc->pathPlanDOFSeq[i];
						config_tmp.push_back(gc->graspMotionSeq[i].jointSeq);
						config_tmp.push_back(gc->graspMotionSeq[i+1].jointSeq);

						bool success = planner.call_planner(config_tmp, gc->pathPlanDOF);
						if(!success) successAll=false;
						planner.call_smoother(config_tmp);


						for(unsigned int j=0; j<config_tmp.size(); j++){
							for(int k=0;k<gc->bodyItemRobot()->body()->numJoints();k++) gc->bodyItemRobot()->body()->joint(k)->q = config_tmp[j][k];
							gc->setInterLink();
							for(int k=0;k<gc->bodyItemRobot()->body()->numJoints();k++) config_tmp[j][k] = gc->bodyItemRobot()->body()->joint(k)->q;
							motionSeq.push_back( MotionState( config_tmp[j], gc->graspMotionSeq[i].graspingState, gc->graspMotionSeq[i].graspingState2,gc->graspMotionSeq[i].id,gc->graspMotionSeq[i].tolerance) );
						}
				}
		}
		else{
				config.push_back(iniJoint);
				config.push_back(finJoint);

				planner.call_planner(config, gc->pathPlanDOF);
				planner.call_smoother(config);

				for(unsigned int j=0; j<config.size(); j++){
						motionSeq.push_back( MotionState( config[j], gc->NOT_GRASPING, gc->NOT_GRASPING) );
				}
		}


		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		gc->object()->p = gc->objVisPos();
		gc->object()->R = gc->objVisRot();

		for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++) gc->bodyItemRobot()->body()->joint(i)->q = finJoint(i);
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





