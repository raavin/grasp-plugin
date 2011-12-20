// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "ManipController.h"

#define m_pi 3.141592
//#define DEBUG_MODE
//#define CALIB_MODE
//#define PUT_PATTERN
//#define OBJ_ENV_CONTACT

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

ManipController::ManipController()  : 	os (MessageView::mainInstance()->cout() ) {
		initPalm = true;
		intention = 0;
		envItem = NULL;
		strategy = RIGHT_RIGHT;
		PlacePlanner::instance()->putPos.Index = 0;
}

ManipController::~ManipController() {
}

ManipController* ManipController::instance() {
		static ManipController* instance = new ManipController();
		return instance;
}

bool ManipController::initial(TargetObject* targetObject, ArmFingers* targetArmFinger)
{
		rb = new RobotLocalFunctions();
		cp = new CollisionPair();
		pf = new ParameterFileData();

		if( grasp::GraspController::initial(targetObject, targetArmFinger) == false) return false;

		chooseArmToGrasp();

		if(initPalm){
				Pinit =  arm_g->arm_path->joint(arm_g->arm_path->numJoints()-1)->p;
				initPalm=false;
		}

		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		if(tc->jointSeq.size()>0){
				jointSeq_ini = tc->jointSeq[0];
				tc->jointSeq.clear();
		}
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
						jointSeq_ini(i) = bodyItemRobot()->body()->joint(i)->q;
		}
		tc->jointSeq.push_back(jointSeq_ini);



		pf->readObjClusterParameters();

		for(list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it != tc->bodyItemEnv.end(); ++it)
				if(PlanBase::instance()->objPressName == (*it)->body()->name())
						envItem = *it;


		if(envItem != NULL){
				Vector3 pressPos = envItem->body()->link(0)->p + envItem->body()->link(0)->attitude()*(PlanBase::instance()->objPressPos);
				//cp->setCollisionObj();
				PlacePlanner::instance()->calcPutPos(pressPos, objVisRot(), Po_put, Ro_put);
				chooseArmToPut();
				selectManipStrategy();
		}

		pf->calcIDPair(intention);

		if     (bodyItemRobot()->body()->name() == "PA10" ||
			 bodyItemRobot()->body()->name() == "PA10_VVV" ) robot=PA10;
		else if(bodyItemRobot()->body()->name() == "HIRO"      ) robot = HIRO;
		else if(bodyItemRobot()->body()->name() == "HRP2"      ) robot = HRP2;

		firstPick = true;

//#ifdef DEBUG_MODE
		os << "Environments: ";
		for(list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it != tc->bodyItemEnv.end(); ++it)
				os << (*it)->body()->name() << " ";
		os << endl;
		os << "Object: " << targetObject->bodyItemObject->body()->name() << endl;
//#endif
		return true;
}

void ManipController::selectManipStrategy()
{
		strategy = RIGHT_RIGHT;

		if     (tc->arm(0)==arm_g && tc->arm(0)==arm_p) strategy = RIGHT_RIGHT;
		else if(tc->arm(1)==arm_g && tc->arm(1)==arm_p) strategy = LEFT_LEFT;
		else if(tc->arm(0)==arm_g && tc->arm(1)==arm_p) strategy = RIGHT_LEFT;
		else if(tc->arm(1)==arm_g && tc->arm(0)==arm_p) strategy = LEFT_RIGHT;

}

void ManipController::chooseArmToGrasp()
{

		Vector3 P = bodyItemRobot()->body()->link(0)->attitude().transpose()*(objVisPos() - bodyItemRobot()->body()->link(0)->p);

		if(tc->arm(1)==NULL || P(1) <0.05){
				arm_g = tc->arm(0);
				fingers_g[0] = tc->fingers(0,0);
				fingers_g[1] = tc->fingers(0,1);
		}else{
				arm_g = tc->arm(1);
				fingers_g[0] = tc->fingers(1,0);
				fingers_g[1] = tc->fingers(1,1);
		}

		fingers_g[0]->fingerGraspPose.resize(fingers_g[0]->fing_path->numJoints());
		fingers_g[1]->fingerGraspPose.resize(fingers_g[1]->fing_path->numJoints());
}

void ManipController::chooseArmToPut()
{
		Vector3 P = bodyItemRobot()->body()->link(0)->attitude().transpose()*(Po_put[0] - bodyItemRobot()->body()->link(0)->p);

		if(tc->arm(1)==NULL || P(1) <0){
				arm_p = tc->arm(0);
				fingers_p[0] = tc->fingers(0,0);
				fingers_p[1] = tc->fingers(0,1);
		}else{
				arm_p = tc->arm(1);
				fingers_p[0] = tc->fingers(1,0);
				fingers_p[1] = tc->fingers(1,1);
		}

		fingers_p[0]->fingerGraspPose.resize(fingers_p[0]->fing_path->numJoints());
		fingers_p[1]->fingerGraspPose.resize(fingers_p[1]->fing_path->numJoints());
}

bool ManipController::withinJointLimit(){

		for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
				if (bodyItemRobot()->body()->joint(i)->ulimit < bodyItemRobot()->body()->joint(i)->q  ||  bodyItemRobot()->body()->joint(i)->llimit > bodyItemRobot()->body()->joint(i)->q)
						return false;

		return true;
}

void ManipController::setTargetObjPos(Vector3& P_ini, Matrix3& R_ini, vector<Vector3>& P_des, vector<Matrix3>& R_des, int j){

		P_des.clear();
		R_des.clear();

		if(strategy==RIGHT_RIGHT || strategy==LEFT_LEFT || strategy==RIGHT_LEFT || strategy==LEFT_RIGHT ){
				P_ini = objVisPos();
				R_ini = objVisRot();
				for(unsigned int i=0; i<Po_put.size(); i++){
						P_des.push_back( Po_put[i] );
						R_des.push_back( Ro_put[i] );
				}
		}
		else if(firstPick && (strategy==RIGHT_PUT_LEFT || strategy==LEFT_PUT_RIGHT)){
				P_ini = objVisPos();
				R_ini = objVisRot();
				P_des.push_back( Po_tmp[j] );
				R_des.push_back( Ro_tmp[j] );
		}
		else if(!firstPick && (strategy==RIGHT_PUT_LEFT || strategy==LEFT_PUT_RIGHT)){
				P_ini = Po_tmp[j];
				R_ini = Ro_tmp[j];
				for(unsigned int i=0; i<Po_put.size(); i++){
						P_des.push_back( Po_put[i] );
						R_des.push_back( Ro_put[i] );
				}
		}

		if(strategy==RIGHT_RIGHT || (firstPick && (strategy==RIGHT_PUT_LEFT || strategy == RIGHT_LEFT)) || (!firstPick && (strategy==LEFT_PUT_RIGHT || strategy == LEFT_RIGHT)) )
				graspingHand = RIGHT;
		else if(strategy==LEFT_LEFT || (!firstPick && (strategy==RIGHT_PUT_LEFT || strategy == RIGHT_LEFT)) || (firstPick && (strategy==LEFT_PUT_RIGHT || strategy == LEFT_RIGHT)) )
				graspingHand = LEFT;


		//Heuristic Rule (to replace)
		if(strategy == RIGHT_LEFT){
				Po_tmp.clear();
				Ro_tmp.clear();
				Vector3 rpyObj = rpyFromRot(R_ini);
				if((fabs(rpyObj(0))<0.001 || fabs(rpyObj(0)-m_pi)<0.001 || fabs(rpyObj(0)+m_pi)<0.001) && (fabs(rpyObj(1))<0.001 || fabs(rpyObj(1)-m_pi)<0.001 || fabs(rpyObj(1)+m_pi)<0.001 )){
						Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p(0)+0.4, arm_g->arm_path->joint(0)->p(1), arm_g->arm_path->joint(1)->p(2)) );
						Ro_tmp.push_back( rotFromRpy(M_PI/2.0, 0, 0)*R_ini );
				}
				else{
						Vector3 rpyObj2 = rpyFromRot(rotFromRpy(M_PI/2.0, 0, 0)*R_ini);
						Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p(0)+0.43, arm_g->arm_path->joint(0)->p(1), arm_g->arm_path->joint(1)->p(2)-0.23) );
						Ro_tmp.push_back( rotFromRpy(M_PI, 0, rpyObj2(2)));
				}

		}
		else if(strategy == LEFT_RIGHT){
				Po_tmp.clear();
				Ro_tmp.clear();
				Vector3 rpyObj = rpyFromRot(R_ini);
				if((fabs(rpyObj(0))<0.001 || fabs(rpyObj(0)-m_pi)<0.001 || fabs(rpyObj(0)+m_pi)<0.001) && (fabs(rpyObj(1))<0.001 || fabs(rpyObj(1)-m_pi)<0.001 || fabs(rpyObj(1)+m_pi)<0.001 )){
						Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p(0)+0.4, arm_g->arm_path->joint(0)->p(1), arm_g->arm_path->joint(1)->p(2)) );
						Ro_tmp.push_back( rotFromRpy(-M_PI/2.0, 0, 0)*R_ini );
				}
				else{
						Vector3 rpyObj2 = rpyFromRot(rotFromRpy(-M_PI/2.0, 0, 0)*R_ini);
						Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p(0)+0.43, arm_g->arm_path->joint(0)->p(1), arm_g->arm_path->joint(1)->p(2)-0.23) );
						Ro_tmp.push_back( rotFromRpy(M_PI, 0, rpyObj2(2)));
				}
		}
}

bool ManipController::searchPickAndPlaceMotion(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_put2, Matrix3& Rp_put2, dvector& th_grasp2, bool edge_grasp)
{
                clock_t start,end;
                start = clock();

		int nj = arm_g->arm_path->numJoints()-1;
		bool ret=false;

		int numG=0;
		if(edge_grasp)
				numG=1;

		//Vector3 Pp_p = arm_p->palm->p;
		//Matrix3 Rp_p = arm_p->palm->attitude();

		double Quality_o=1.0e+100;

		Vector3 Po_ini, Pp_grasp, Pp_put;
		Matrix3 Ro_ini, Rp_grasp, Rp_put;
		vector<Vector3> Po_des;
		vector<Matrix3> Ro_des;

		setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

		for(unsigned int id=0; id<pf->idPairs.size(); id++){

				int id0 = pf->idPairs[id][0];
				int id1 = pf->idPairs[id][1];

				for(unsigned int c0=0; c0<pf->objClusters[id0].controlPoints.size(); c0++){
				for(unsigned int c1=0; c1<pf->objClusters[id1].controlPoints.size(); c1++){

						Vector3 oPco1 = pf->objClusters[id0].controlPoints[c0];
						Vector3 oPco2 = pf->objClusters[id1].controlPoints[c1];
						Vector3 oNco1 = pf->objClusters[id0].normal;
						Vector3 oNco2 = pf->objClusters[id1].normal;

						Vector3 pPcr1 = pf->handClusters[0].controlPoints[numG] ;
						Vector3 pNcr1 = pf->handClusters[0].normal;
						Vector3 pTcr1 = pf->handClusters[0].approachVec[0];
						Matrix3 pRcr1 = v3(pNcr1, pTcr1, cross(pNcr1,pTcr1) );

						if(norm2(normalPoint(oPco2, oPco1, oNco1)-oPco2)>1.0e-3) continue;

						for(unsigned int ap=0; ap<pf->objClusters[id0].approachVec.size(); ap++){

								Vector3 oTco1 =  pf->objClusters[id0].approachVec[ap];
								Vector3 oTco2 =  pf->objClusters[id1].approachVec[ap];

								if(norm2(oTco1)<1.0e-3) continue;
								if(norm2(oTco2)<1.0e-3) continue;
								if(!included(ap, pf->graspPostureCandidate)) continue;

								Matrix3 oRco1 = v3(oNco1, oTco1, cross(oNco1,oTco1) );
								Matrix3 oRco2 = v3(oNco2, oTco2, cross(oNco2,oTco2) );

								Vector3 eco1 =  Ro_ini*oTco1;
								Vector3 ncr1 = -Ro_ini*oNco1;
								Matrix3 R_psi = rodrigues(eco1, m_pi);

								Rp_grasp = R_psi*Ro_ini*oRco1*trans(pRcr1);

								Vector3 Pco1 = Po_ini + Ro_ini*oPco1;
								Vector3 Pco2 = Po_ini + Ro_ini*oPco2;

								Vector3 nco1 = -ncr1;
								Pco2 = Pco1 + nco1 * dot(nco1, Pco2-Pco1);

								VectorXd th_grasp(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());
								VectorXd th_put(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());

								rb->calcPalmFingParam(robot, Pco1, Pco2, Rp_grasp, pPcr1, pNcr1, pTcr1, Pp_grasp, th_grasp);

								//if( ! arm_p->IK_arm(Pp_p,     arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_p),  0.01) ) continue;
								if( ! arm_g->IK_arm(Pp_grasp, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp),  0.0) ){
#ifdef DEBUG_MODE
										cout << "IK(grasping posture) not solvable" << endl;
#endif
										continue;}

								int k=0;
								for(int i=0; i<2; i++)
										for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
												fingers_g[i]->fing_path->joint(j)->q = th_grasp(k++);

								double nom = 1.0-fabs(dot(Rp_grasp*Vector3(0.0,0.0,1.0), Vector3(Pp_grasp - Pinit)/norm2(Pp_grasp - Pinit)));

								double Quality = pf->gainParameter[0]*((c0+1) + (c1+1) + id0 + id1) + pf->gainParameter[1]*nom;
								//(c0+1):ctrlPtsP is ordered by the distance from the center. surf?: cluster is ordered by the area.

								if( !( Quality < Quality_o && withinJointLimit() )) continue;

								if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING) || !withinJointLimit()){
#ifdef DEBUG_MODE
										cout << "Collision(grasping posture)" << endl;
#endif
										continue;}

								//Collision of putting posture
								if(envItem != NULL){

										bool dist = false;
										for(unsigned int env=0; env<Po_des.size(); env++){

												Vector3 eco2 =  Ro_des[env]*oTco1;
												R_psi = rodrigues(eco2, m_pi);
												Rp_put = R_psi*Ro_des[env]*oRco1*trans(pRcr1);

												Pco1 = Po_des[env] + Ro_des[env]*oPco1;
												Pco2 = Po_des[env] + Ro_des[env]*oPco2;

												rb->calcPalmFingParam(robot, Pco1, Pco2, Rp_put, pPcr1, pNcr1, pTcr1, Pp_put, th_put);

												if( ! arm_p->IK_arm(Pp_put,  arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_put),  0.0) ) continue;

												//k=0;
												//for(int i=0; i<2; i++)
												//		for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++)
												//				fingers_p[i]->fing_path->joint(j)->q = th_put(k++);

												if(!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING) && withinJointLimit()){
														dist=true;
														break;
												}
										}

										if(!dist){
#ifdef DEBUG_MODE
												cout << "Collision(putting posture)" << endl;
#endif
												continue;}

										if(!calcJointSeqTest(Pp_grasp, Rp_grasp, Pp_put, Rp_put, eco1)) continue;
								}


#ifdef DEBUG_MODE
								cout << "Solution found (Cluster= " << id0 << "/" << id1 << ", ControlPoint= " << c0 << "/" << c1 << ", Approach= " << ap << ")" << endl;
#endif
								Quality_o = Quality;
								Pp_grasp2 = Pp_grasp;
								Rp_grasp2 = Rp_grasp;
								Pp_put2 = Pp_put;
								Rp_put2 = Rp_put;
								th_grasp2 = th_grasp;
								approachVec = eco1;
								ret = true;
								tc->flush();
						}
						//tc->flush();
						//usleep(1000000);
				}
				}
		}

		if(ret){
				arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2), 0.0);

				int k=0;
				for(int i=0; i<2; i++)
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
								fingers_g[i]->fing_path->joint(j)->q = th_grasp2(k++);

				bodyItemRobot()->body()->calcForwardKinematics();
		}


		end = clock();
                cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		tc->flush();

		return ret;
}

bool ManipController::searchPickAndPlaceWithRegrasp(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_tmp_put2, Matrix3& Rp_tmp_put2, Vector3& Pp_regrasp2, Matrix3& Rp_regrasp2, Vector3& Pp_put2, Matrix3& Rp_put2, VectorXd& th_grasp2, VectorXd& th_put2, bool edge_grasp)
{

		int nj = arm_g->arm_path->numJoints()-1;
		bool ret=false;

		int numG=0;
		if(edge_grasp)
				numG=1;

		Vector3 Pp_ini[2];
		Matrix3 Rp_ini[2];
		Pp_ini[0] = arm_g->palm->p;
		Pp_ini[1] = arm_p->palm->p;
		Rp_ini[0] = arm_g->palm->attitude();
		Rp_ini[1] = arm_p->palm->attitude();

		VectorXd jp(bodyItemRobot()->body()->numJoints());
		for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
				jp(i) = bodyItemRobot()->body()->joint(i)->q;

		double Quality_o=1.0e+100;

		Vector3 Po_ini, Pp_grasp, Pp_put;
		Matrix3 Ro_ini, Rp_grasp, Rp_put;
		vector<Vector3> Po_des;
		vector<Matrix3> Ro_des;

		setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

		for(unsigned int ida=0; ida<pf->idPairs.size(); ida++){
		for(unsigned int idb=0; idb<pf->idPairs.size(); idb++){

				int id0 = pf->idPairs[ida][0];
				int id1 = pf->idPairs[ida][1];
				int id2 = pf->idPairs[idb][0];
				int id3 = pf->idPairs[idb][1];

				if(id0==id2 || id0==id3) continue;

				for(unsigned int c0=0; c0<pf->objClusters[id0].controlPoints.size(); c0++){
				for(unsigned int c1=0; c1<pf->objClusters[id1].controlPoints.size(); c1++){

						Vector3 oPco1 = pf->objClusters[id0].controlPoints[c0];
						Vector3 oPco2 = pf->objClusters[id1].controlPoints[c1];
						Vector3 oNco1 = pf->objClusters[id0].normal;
						Vector3 oNco2 = pf->objClusters[id1].normal;
						Vector3 pPcr1 = pf->handClusters[0].controlPoints[numG] ;
						Vector3 pNcr1 = pf->handClusters[0].normal;
						Vector3 pTcr1 = pf->handClusters[0].approachVec[0];
						Matrix3 pRcr1 = v3(pNcr1, pTcr1, cross(pNcr1,pTcr1) );
						if(norm2(normalPoint(oPco2, oPco1, oNco1)-oPco2)>1.0e-3) continue;

						for(unsigned int c2=0; c2<pf->objClusters[id2].controlPoints.size(); c2++){
						for(unsigned int c3=0; c3<pf->objClusters[id3].controlPoints.size(); c3++){

								Vector3 oPco3 = pf->objClusters[id2].controlPoints[c2];
								Vector3 oPco4 = pf->objClusters[id3].controlPoints[c3];
								Vector3 oNco3 = pf->objClusters[id2].normal;
								Vector3 oNco4 = pf->objClusters[id3].normal;
								if(norm2(normalPoint(oPco4, oPco3, oNco3)-oPco4)>1.0e-3) continue;

								for(unsigned int ap0=0; ap0<pf->objClusters[id0].approachVec.size(); ap0++){

										Vector3 oTco1 =  pf->objClusters[id0].approachVec[ap0];
										Vector3 oTco2 =  pf->objClusters[id1].approachVec[ap0];

										if(norm2(oTco1)<1.0e-3) continue;
										if(norm2(oTco2)<1.0e-3) continue;
										if(!included(ap0, pf->graspPostureCandidate)) continue;

										Matrix3 oRco1 = v3(oNco1, oTco1, cross(oNco1,oTco1) );
										Matrix3 oRco2 = v3(oNco2, oTco2, cross(oNco2,oTco2) );
										Vector3 eco1 =  Ro_ini*oTco1;
										Vector3 eco2 =  Ro_tmp[0]*oTco1;
										Vector3 ncr1 = -Ro_ini*oNco1;

										Matrix3 R_psi1 = rodrigues(eco1, m_pi);
										Rp_grasp = R_psi1*Ro_ini*oRco1*trans(pRcr1);

										Matrix3 R_psi2 = rodrigues(eco2, m_pi);
										Matrix3 Rp_tmp_put = R_psi2*Ro_tmp[0]*oRco1*trans(pRcr1);

										for(unsigned int ap1=0; ap1<pf->objClusters[id2].approachVec.size(); ap1++){

												if(!(id0==5 && id1==4 && id2==2 && id3==3)) continue;

												Vector3 oTco3 =  pf->objClusters[id2].approachVec[ap1];
												Vector3 oTco4 =  pf->objClusters[id3].approachVec[ap1];

												if(norm2(oTco3)<1.0e-3) continue;
												if(norm2(oTco4)<1.0e-3) continue;
												if(!included(ap1, pf->graspPostureCandidate)) continue;

												Matrix3 oRco3 = v3(oNco3, oTco3, cross(oNco3,oTco3) );
												Matrix3 oRco4 = v3(oNco4, oTco4, cross(oNco4,oTco4) );
												Vector3 eco3 =  Ro_tmp[0]*oTco3;
												Vector3 ncr3 = -Ro_tmp[0]*oNco3;

												Matrix3 R_psi3 = rodrigues(eco3, m_pi);
												Matrix3 Rp_regrasp = R_psi3*Ro_tmp[0]*oRco3*trans(pRcr1);

												Vector3 Pco1 = Po_ini + Ro_ini*oPco1;
												Vector3 Pco2 = Po_ini + Ro_ini*oPco2;

												Vector3 nco1 = -ncr1;
												Pco2 = Pco1 + nco1 * dot(nco1, Pco2-Pco1);

												Vector3 Pco3 = Po_tmp[0] + Ro_tmp[0]*oPco3;
												Vector3 Pco4 = Po_tmp[0] + Ro_tmp[0]*oPco4;

												Vector3 nco3 = -ncr3;
												Pco4 = Pco3 + nco3 * dot(nco3, Pco4-Pco3);

												int nJ = fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints();
												VectorXd th_grasp(nJ), th_tmp1(nJ), th_tmp2(nJ), th_put(nJ);

												Vector3 Pp_tmp_put, Pp_regrasp;
												rb->calcPalmFingParam(robot, Pco1, Pco2, Rp_grasp, pPcr1, pNcr1, pTcr1, Pp_grasp, th_grasp);
												rb->calcPalmFingParam(robot, Pco3, Pco4, Rp_regrasp,  pPcr1, pNcr1, pTcr1, Pp_regrasp,  th_tmp2);

												Pco1 = Po_tmp[0] + Ro_tmp[0]*oPco1;
												Pco2 = Po_tmp[0] + Ro_tmp[0]*oPco2;

												rb->calcPalmFingParam(robot, Pco1, Pco2, Rp_tmp_put, pPcr1, pNcr1, pTcr1, Pp_tmp_put, th_tmp1);

												//double nom = 1.0-dot(Vector3(Rp_tmp_put(0,0), Rp_tmp_put(1,0), Rp_tmp_put(2,0) ), Vector3(Rp_regrasp(0,0), Rp_regrasp(1,0), Rp_regrasp(2,0) ) );
												//double Quality = nom; //pf->gainParameter[0]*((c0+1) + (c1+1) + id0 + id1) + pf->gainParameter[0]*((c2+1) + (c3+1) + id2 + id3);
												//if( Quality > Quality_o || ret) continue;

												bool cond = true;
												if(! arm_g->IK_arm(Pp_grasp,  arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp), 0.0) ) cond = false;
												if(! arm_p->IK_arm(Pp_ini[1], arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_ini[1]), arm_g->arm_path->joint(0)->q) ) cond = false;
												if(!cond){
#ifdef DEBUG_MODE
														cout << "IK(grasping posture) not solvable" << endl;
#endif
														continue;}


												int k=0;
												for(int i=0; i<2; i++)
														for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
																fingers_g[i]->fing_path->joint(j)->q = th_grasp(k++);

												if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING) || !withinJointLimit()){
#ifdef DEBUG_MODE
														cout << "Collision (grasping posture) occurs" << endl;
#endif
														continue;}

												if( ! arm_g->IK_arm(Pp_tmp_put,  arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_tmp_put),  0.001) ) cond = false;
												if( ! arm_p->IK_arm(Pp_regrasp,  arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_regrasp),  0.001) ) cond = false;
												if(!cond){
#ifdef DEBUG_MODE
														cout << "IK (regrasping posture) not solvable" << endl;
#endif
														continue;}


												k=0;
												for(int i=0; i<2; i++)
														for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++)
																fingers_p[i]->fing_path->joint(j)->q = th_tmp2(k++);


												if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING) || !withinJointLimit()){
#ifdef DEBUG_MODE
														cout << "Collision (regrasping posture) occurs" << endl;
#endif
														continue;}


												cond = false;
												for(unsigned int env=0; env<Po_des.size(); env++){

														Vector3 eco4 =  Ro_des[env]*oTco3;
														Matrix3 R_psi4 = rodrigues(eco4, m_pi);
														Rp_put  = R_psi4*Ro_des[env]*oRco3*trans(pRcr1);

														Pco3 = Po_des[env] + Ro_des[env]*oPco3;
														Pco4 = Po_des[env] + Ro_des[env]*oPco4;

														rb->calcPalmFingParam(robot, Pco3, Pco4, Rp_put,  pPcr1, pNcr1, pTcr1, Pp_put,  th_put);

														if( ! arm_p->IK_arm(Pp_put,  arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_put),  0.0) ) continue;
														if( ! arm_g->IK_arm(Pp_tmp_put,  arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_tmp_put), arm_p->arm_path->joint(0)->q) ) continue;

														if(!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING)){
																cond = true;
																break;
														}
												}
												if(!cond){
#ifdef DEBUG_MODE
														cout << "Collision (regrasping posture) occurs" << endl;
#endif
														continue;}

												if( !calcJointSeqTestRegrasp(Pp_grasp, Rp_grasp, Pp_tmp_put, Rp_tmp_put, Pp_regrasp, Rp_regrasp, Pp_put, Rp_put, eco1, eco3)) continue;

												double Quality = pf->gainParameter[0]*((c0+1) + (c1+1) + id0 + id1) + pf->gainParameter[0]*((c2+1) + (c3+1) + id2 + id3);

												if( Quality > Quality_o) continue;

												cout << "Found solition " << id0 << id1 << id2 << id3 << " " << Quality << endl;
												Quality_o = Quality;
												Pp_grasp2 = Pp_grasp;
												Rp_grasp2 = Rp_grasp;
												Pp_tmp_put2 = Pp_tmp_put;
												Rp_tmp_put2 = Rp_tmp_put;
												Pp_regrasp2 = Pp_regrasp;
												Rp_regrasp2 = Rp_regrasp;
												Pp_put2 = Pp_put;
												Rp_put2 = Rp_put;
												th_grasp2 = th_grasp;
												th_put2 = th_tmp2;
												approachVec = eco1;
												approachVec2 = eco3;
												ret = true;
												tc->flush();
										}
								}
						}
						}
				}
				}
		}
		}

		if(ret){
				arm_g->IK_arm(Pp_tmp_put2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_tmp_put2), 0.0);
				arm_p->IK_arm(Pp_regrasp2, arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_regrasp2), arm_g->arm_path->joint(0)->q);

				int k=0;
				for(int i=0; i<2; i++)
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
								fingers_g[i]->fing_path->joint(j)->q = th_grasp2(k++);

				k=0;
				for(int i=0; i<2; i++)
						for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++)
								fingers_p[i]->fing_path->joint(j)->q = th_put2(k++);
		}
		else{

				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
						bodyItemRobot()->body()->joint(i)->q = jp(i);
		}

		bodyItemRobot()->body()->calcForwardKinematics();

		tc->flush();

		return ret;
}

bool ManipController::isColliding(int graspingState, int graspingState2)
{
		if(graspingHand==RIGHT){
				tc->setGraspingState(graspingState);
				tc->setGraspingState2(graspingState2);
		}
		else{
				tc->setGraspingState2(graspingState);
				tc->setGraspingState(graspingState2);
		}

		tc->calcForwardKinematics();

		bool ret = tc->isColliding();

		if(ret && (tc->colPairName[0]=="LARM_JOINT1" && tc->colPairName[1]=="WAIST"))
				ret = false;

		if(ret && (tc->colPairName[0]=="CHEST_JOINT0" && tc->colPairName[1]=="LARM_JOINT1"))
				ret = false;

#ifdef DEBUG_MODE
		if(ret)cout << "Collision between " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;
#endif


		return ret;
}

void ManipController::calcArmJointSeq(VectorXd& armJointSeq, ArmPtr& arm_)
{
		for(int i=0; i<arm_->arm_path->numJoints(); i++)
				armJointSeq(i) = tc->jointSeq.back()(arm_->arm_path->joint(i)->jointId);
}

void ManipController::setGraspingStateSeq(int graspingState, int graspingState2, int contactState)
{
		if(graspingHand==RIGHT){
				tc->setGraspingState(graspingState);
				tc->setGraspingState2(graspingState2);
		}
		else{
				tc->setGraspingState2(graspingState);
				tc->setGraspingState(graspingState2);
		}

		tc->graspingStateSeq.push_back(tc->getGraspingState());
		tc->graspingStateSeq2.push_back(tc->getGraspingState2());

#ifdef OBJ_ENV_CONTACT
		tc->setObjectContactState(contactState);
		tc->objectContactStateSeq.push_back(tc->getObjectContactState());
#endif
}

void ManipController::setJointSeq(int graspingState, VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[])
{
		for(int i=0; i<arm_->arm_path->numJoints(); i++)
				jointSeq(arm_->arm_path->joint(i)->jointId) = arm_->arm_path->joint(i)->q;

		for(int i=0; i<2; i++)
				for(int j=0; j<fingers_[i]->fing_path->numJoints(); j++){
						if(graspingState == PlanBase::GRASPING)
								jointSeq(fingers_[i]->fing_path->joint(j)->jointId) = fingers_[i]->fingerGraspPose[j];
						else
								jointSeq(fingers_[i]->fing_path->joint(j)->jointId) = fingers_[i]->fingerOpenPose[j];

						fingers_[i]->fing_path->joint(j)->q = jointSeq(fingers_[i]->fing_path->joint(j)->jointId);
				}
}

bool ManipController::setMotionSeq(int graspingState, int contactState, const Vector3& P, const Matrix3& R, VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time)
{
		VectorXd armJointSeq(arm_->arm_path->numJoints());

		calcArmJointSeq(armJointSeq, arm_);

		if(!arm_->IK_arm(P,  R, 0.0, armJointSeq)) return false;

		setJointSeq(graspingState, jointSeq, arm_, fingers_);
		tc->jointSeq.push_back(jointSeq);

		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);

		setGraspingStateSeq(graspingState, PlanBase::NOT_GRASPING, contactState);
		//cout << "motionTimeSeq << " << time << endl;
		tc->motionTimeSeq.push_back(time);

		return true;
}

bool ManipController::setMotionSeqDual(int graspingState, int graspingState2, int contactState, const Vector3& P, const Matrix3& R, const Vector3& P2, const Matrix3& R2, VectorXd& jointSeq, ArmPtr& arm, FingerPtr fingers[], double time)
{
		ArmPtr arm2;
		if(arm == arm_g)	arm2 = arm_p;
		else			arm2 = arm_g;

		FingerPtr fingers2[2];
		if(fingers[0] == fingers_g[0]) for(int i=0; i<2; i++) fingers2[i] = fingers_p[i];
		else                           for(int i=0; i<2; i++) fingers2[i] = fingers_g[i];

		if(!arm->IK_arm(P,  R, 0.0)) return false;
		if(!arm2->IK_arm(P2,  R2, arm->arm_path->joint(0)->q)) return false;

		setJointSeq(graspingState, jointSeq, arm, fingers);
		setJointSeq(graspingState2, jointSeq, arm2, fingers2);
		tc->jointSeq.push_back(jointSeq);

		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);

		setGraspingStateSeq(graspingState, graspingState2, contactState);

		tc->motionTimeSeq.push_back(time);

		return true;
}

bool ManipController::calcJointSeqTest(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp_put, Matrix3& Rp_put, Vector3& appVec){

		bool cond = true;
		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->gainParameter[2], v_margin=pf->gainParameter[3], lift_height=pf->gainParameter[4], release_height=pf->gainParameter[5];
		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		for(int i=0; i<2; i++){
				for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
						fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q;
		}  

		int nj = arm_g->arm_path->numJoints()-1;

		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		tc->jointSeq.clear();
		tc->graspingStateSeq.clear();
		tc->graspingStateSeq2.clear();
		tc->pathPlanDOFSeq.clear();
		tc->objectContactStateSeq.clear();
		tc->setTrajectoryPlanDOF(graspingHand);
		tc->motionTimeSeq.clear();

		//===== Initial Point (already included in armJointSeq)
		/*
		for(int i=0; i<arm_g->arm_path->numJoints(); i++)
					arm_g->arm_path->joint(i)->q = jointSeq(arm_g->arm_path->joint(i)->jointId);
		for(int i=0; i<arm_p->arm_path->numJoints(); i++)
					arm_p->arm_path->joint(i)->q = jointSeq(arm_p->arm_path->joint(i)->jointId);

		bodyItemRobot()->body()->calcForwardKinematics();
		*/

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==== Mid Point
		if(robot==PA10){
				for(int i=0; i<arm_g->arm_path->numJoints(); i++)
						arm_g->arm_path->joint(i)->q = jointSeq(arm_g->arm_path->joint(i)->jointId);
				arm_g->arm_path->calcForwardKinematics();

				Vector3 Pp_mid = (arm_g->arm_path->joint(nj)->p + Vector3(Pp_grasp - app_length*appVec + v_margin*z))*0.5;
				Matrix3 Att_mid = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);

				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_mid, Att_mid, jointSeq, arm_g, fingers_g, mtime[0])){
#ifdef DEBUG_MODE
						cout << "Mid point not solvable" << endl;
#endif
						return false;}
		}

		//==== Approach Point
		Vector3 Pp_app = Pp_grasp - app_length*appVec + v_margin*z;
 		Matrix3 Att_grasp = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);
		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_app, Att_grasp, jointSeq, arm_g, fingers_g, mtime[0])){
#ifdef DEBUG_MODE
				cout << "Approach point not solvable" << endl;
#endif
				return false;}

		//==== Grasping Point
		if(!setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(!setMotionSeq(PlanBase::GRASPING,       PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Grasping point not solvable" << endl;
#endif
				return false;}

		//==== LiftUp Point
		Vector3 Pp_lift = Pp_grasp+lift_height*z;
		if(!setMotionSeq(PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_lift, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Liftup point not solvable" << endl;
#endif
				return false;}

		//=== Top of release point
		Vector3 Pp_rel = Pp_put+release_height*z;
		Matrix3 Att_put = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_put);
		if(!setMotionSeq(PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_rel, Att_put, jointSeq, arm_g, fingers_g, mtime[0])) cond = false;
		if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Top of release point not solvable" << endl;
#endif
				return false;}

		//== Release point
		if(!setMotionSeq(PlanBase::GRASPING,     PlanBase::OFF_ENVIRONMENT, Pp_put, Att_put, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_put, Att_put, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Release point not solvable" << endl;
#endif
				return false;}


		//== Return to original position
		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_rel, Att_put, jointSeq, arm_g, fingers_g, mtime[1])) return false;
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//==Just for Graphics
		tc->setGraspingState(PlanBase::NOT_GRASPING);
		arm_g->IK_arm(Pp_grasp, Att_grasp, 0.0);
		bodyItemRobot()->body()->calcForwardKinematics();
		

		bool limit = true;
		if(robot==PA10)
				for(unsigned int i=1; i<tc->jointSeq.size(); i++)
						if( tc->jointSeq[i](arm_g->arm_path->joint(4)->jointId)+m_pi > arm_g->arm_path->joint(4)->ulimit)
								limit=false;
		return limit;
}

bool ManipController::calcJointSeqTestRegrasp(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp1, Matrix3& Rp1, Vector3& Pp2, Matrix3& Rp2, Vector3& Pp_put, Matrix3& Rp_put, Vector3& appVec1, Vector3& appVec2 ){

		bool cond = true;
		Vector3 z(0.0, 0.0, 1.0);

		double app_length     = pf->gainParameter[2];
		double lift_height    = pf->gainParameter[4];
		double release_height = pf->gainParameter[5];
		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		for(int i=0; i<2; i++){
				for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
						fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q;
						fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q;
				}
		}  

		int nj = arm_g->arm_path->numJoints()-1;

		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		tc->jointSeq.clear();
		tc->pathPlanDOFSeq.clear();
		tc->graspingStateSeq.clear();
		tc->graspingStateSeq2.clear();
		tc->objectContactStateSeq.clear();
		tc->setTrajectoryPlanDOF(graspingHand);
		tc->motionTimeSeq.clear();

		//===== Initial Point (already included in armJointSeq)

		//for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
		//		bodyItemRobot()->body()->joint(i)->q = jointSeq(i);
		//bodyItemRobot()->body()->calcForwardKinematics();

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==== Approach Point
		//Vector3 Pp_app = Pp_grasp - margin*Rp_grasp*z + v_margin*z;
		Vector3 Pp_app = Pp_grasp - app_length*appVec1;
 		Matrix3 Att_grasp = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);

		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_app, Att_grasp, jointSeq, arm_g, fingers_g, mtime[0])){
#ifdef DEBUG_MODE
				cout << "Approach point not solvable" << endl;
#endif
				return false;}

		//==== Grasping Point
		if(!setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(!setMotionSeq(PlanBase::GRASPING,       PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Grasping point 1 not solvable" << endl;
#endif
				return false;}


		//==== LiftUp Point
		Vector3 Pp_lift = Pp_grasp+lift_height*z;
		if(!setMotionSeq(PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_lift, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])){
#ifdef DEBUG_MODE
				cout << "Liftup point 1 not solvable" << endl;
#endif
				return false;}

		//== Release point
		Matrix3 Att_put = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp1);
		Pp_app = Pp2 - app_length*appVec2;
 		Att_grasp = arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp2);

		//if(!setMotionSeq(PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pp1, Att_put, jointSeq, arm_g, fingers_g, mtime[0])){
		if(cond && !setMotionSeqDual(PlanBase::GRASPING, PlanBase::NOT_GRASPING,     PlanBase::OFF_ENVIRONMENT, Pp1,    Att_put, Pp_app, Att_grasp, jointSeq, arm_g, fingers_g, mtime[0])){
#ifdef DEBUG_MODE
				cout << "Release point 1 not solvable" << endl;
#endif
				return false;}

		int graspingHand_org = graspingHand;

		if(strategy==LEFT_RIGHT)	graspingHand = RIGHT;
		else				graspingHand = LEFT;

		tc->setTrajectoryPlanDOF(graspingHand);

		//==== Approach Point
		//Pp_app = Pp2 - app_length*appVec2;
 		//Att_grasp = arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp2);
		//if(cond && !setMotionSeqDual(PlanBase::UNDER_GRASPING, PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_app, Att_grasp, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[0])) cond = false;

		//==== Grasping Point

		if(cond && !setMotionSeqDual(PlanBase::UNDER_GRASPING, PlanBase::GRASPING,     PlanBase::OFF_ENVIRONMENT, Pp2,    Att_grasp, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[1])) cond = false;
		if(cond && !setMotionSeqDual(PlanBase::GRASPING,       PlanBase::GRASPING,     PlanBase::OFF_ENVIRONMENT, Pp2,    Att_grasp, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[2])) cond = false;
		if(cond && !setMotionSeqDual(PlanBase::GRASPING,       PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT, Pp2,    Att_grasp, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[2])) cond = false;
		if(cond && !setMotionSeqDual(PlanBase::GRASPING,       PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_app, Att_grasp, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[1])) cond = false;
#ifdef DEBUG_MODE
		if(!cond) cout << "Grasping point 2 not solvable" << endl;
#endif

		//=== Top of release point
		Vector3 Pp_rel = Pp_put+release_height*z;
		Matrix3 Att_put2 = arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_put);
		if(cond && !setMotionSeqDual(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT, Pp_rel, Att_put2, Pp1, Att_put, jointSeq, arm_p, fingers_p, mtime[0])) cond = false;
#ifdef DEBUG_MODE
		if(!cond) cout << "Top of release point 2 not solvable" << endl;
#endif

		//== Release point
		if(cond && !setMotionSeq(PlanBase::GRASPING,     PlanBase::OFF_ENVIRONMENT, Pp_put, Att_put2, jointSeq, arm_p, fingers_p, mtime[1])) cond = false;
		if(cond && !setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT,  Pp_put, Att_put2, jointSeq, arm_p, fingers_p, mtime[2])) cond = false;
#ifdef DEBUG_MODE
		if(!cond) cout << "Release point 2 not solvable" << endl;
#endif

		//== Return to original position

		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_rel, Att_put2, jointSeq, arm_p, fingers_p, mtime[1])) cond = false;
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		graspingHand = graspingHand_org;

		//==Just for Graphics
		tc->setGraspingState(PlanBase::NOT_GRASPING);
		arm_p->IK_arm(Pp_put, Att_put2, 0.0);
		bodyItemRobot()->body()->calcForwardKinematics();

		return cond;

}

bool ManipController::movePallet()
{

		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);


		Vector3 tPr(0.0,  0.15, 0.015);
		Vector3 tPl(0.0, -0.15, 0.015);

		Vector3 Pt = targetObject->bodyItemObject->body()->link(0)->p;
		Matrix3 Rt = targetObject->bodyItemObject->body()->link(0)->attitude();

		Vector3 pPcr1 = pf->handClusters[0].controlPoints[0];
		Vector3 pNcr1 = pf->handClusters[0].normal;
		Vector3 pTcr1 = pf->handClusters[0].approachVec[0];
		Matrix3 pRcr1 = v3(pNcr1, pTcr1, cross(pNcr1,pTcr1) );

		fingers_g[0] = tc->fingers(0,0);
		fingers_g[1] = tc->fingers(0,1);
		fingers_p[0] = tc->fingers(1,0);
		fingers_p[1] = tc->fingers(1,1);
		arm_g = tc->arm(0);
		arm_p = tc->arm(1);

		VectorXd theta(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());
		VectorXd theta2(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());
		VectorXd armSeq1(arm_g->arm_path->numJoints()), armSeq2(arm_p->arm_path->numJoints());
		VectorXd armSeq3(arm_g->arm_path->numJoints()), armSeq4(arm_p->arm_path->numJoints());

		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->gainParameter[2];
		int nj = arm_g->arm_path->numJoints()-1;

		Vector3 Ppr, Ppl;
		Matrix3 Rpr=arm_g->arm_path->joint(nj)->calcRfromAttitude(rotFromRpy(0.0, -1.5708, -M_PI/4.0));
		Matrix3 Rpl=arm_p->arm_path->joint(nj)->calcRfromAttitude(rotFromRpy(0.0, -1.5708, -M_PI/4.0));

		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		tc->jointSeq.clear();
		tc->pathPlanDOFSeq.clear();
		tc->objectContactStateSeq.clear();
		tc->graspingStateSeq.clear();
		tc->graspingStateSeq2.clear();
		tc->setTrajectoryPlanDOF(graspingHand);
		tc->motionTimeSeq.clear();

		//===== Initial Point (already included in armJointSeq)
		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		jointSeq(0) -= M_PI/4.0;
		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//==== Approach Point
		Vector3 P1 = Pt + Rt*tPr + app_length*z;
		Vector3 P2 = Pt + Rt*tPl + app_length*z;
		Vector3 Pr=P1, Pl=P2;
		if(P1(1)>P2(1)){
				Pr = P2;
				Pl = P1;
		}

		rb->calcPalmFingParam(robot, Pr, Pr, Rpr,  pPcr1, pNcr1, pTcr1, Ppr,  theta);
		rb->calcPalmFingParam(robot, Pl, Pl, Rpl,  pPcr1, pNcr1, pTcr1, Ppl,  theta2);

		for(int i=0; i<2; i++)
				for(int j=0; j<2; j++){
						tc->fingers(i,j)->fingerGraspPose.resize(tc->fingers(i,j)->fing_path->numJoints());
						for(int k=0; k<tc->fingers(i,j)->fing_path->numJoints(); k++)
								tc->fingers(i,j)->fingerGraspPose[k] = theta[2*j+k];
				}

		if(!setMotionSeqDual(PlanBase::UNDER_GRASPING, PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Ppr, Rpr, Ppl, Rpl, jointSeq, arm_g, fingers_g, mtime[0]))  return false;

		calcArmJointSeq(armSeq1, arm_g);
		calcArmJointSeq(armSeq2, arm_p);

		//==== Grasping Point
		Pr -= app_length*z;
		Pl -= app_length*z;

		rb->calcPalmFingParam(robot, Pr, Pr, Rpr,  pPcr1, pNcr1, pTcr1, Ppr,  theta);
		rb->calcPalmFingParam(robot, Pl, Pl, Rpl,  pPcr1, pNcr1, pTcr1, Ppl,  theta2);

		if(!setMotionSeqDual(PlanBase::UNDER_GRASPING, PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Ppr, Rpr, Ppl, Rpl, jointSeq, arm_g, fingers_g, mtime[1]))  return false;

		for(int i=0; i<2; i++)
				for(int j=0; j<2; j++)
						for(int k=0; k<tc->fingers(i,j)->fing_path->numJoints(); k++)
								tc->fingers(i,j)->fing_path->joint(k)->q = theta[2*j+k];

		if(!setMotionSeqDual(PlanBase::GRASPING,       PlanBase::GRASPING, PlanBase::ON_ENVIRONMENT, Ppr, Rpr, Ppl, Rpl, jointSeq, arm_g, fingers_g, mtime[2]))  return false;

		calcArmJointSeq(armSeq3, arm_g);
		calcArmJointSeq(armSeq4, arm_p);

		//==== LiftUp Point
		for(int i=0; i<armSeq1.size(); i++)
				jointSeq(tc->arm(0)->arm_path->joint(i)->jointId) = armSeq1(i);
		for(int i=0; i<armSeq2.size(); i++)
				jointSeq(tc->arm(1)->arm_path->joint(i)->jointId) = armSeq2(i);

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::GRASPING, PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT );
		tc->motionTimeSeq.push_back(mtime[1]);

		//=== Top of release point

		jointSeq(0) -= M_PI/4.0;

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::GRASPING, PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//== Release point

		for(int i=0; i<armSeq3.size(); i++)
				jointSeq(tc->arm(0)->arm_path->joint(i)->jointId) = armSeq3(i);
		for(int i=0; i<armSeq4.size(); i++)
				jointSeq(tc->arm(1)->arm_path->joint(i)->jointId) = armSeq4(i);

		jointSeq(0) -= M_PI/4.0;

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::GRASPING, PlanBase::GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		for(int i=0; i<2; i++)
				for(int j=0; j<2; j++)
						for(int k=0; k<tc->fingers(i,j)->fing_path->numJoints(); k++)
								jointSeq(tc->fingers(i,j)->fing_path->joint(k)->jointId) = tc->fingers(i,j)->fingerOpenPose[k];

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[2]);

		//==Top of release point
		for(int i=0; i<armSeq1.size(); i++)
				jointSeq(tc->arm(0)->arm_path->joint(i)->jointId) = armSeq1(i);
		for(int i=0; i<armSeq2.size(); i++)
				jointSeq(tc->arm(1)->arm_path->joint(i)->jointId) = armSeq2(i);

		jointSeq(0) -= M_PI/4.0;

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==Return to initial
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//---------
		bodyItemRobot()->body()->calcForwardKinematics();

		rb->writeFile(robot, tc->jointSeq, tc->motionTimeSeq);

		return true;
}

Matrix3 upright(const Matrix3& R)
{
		if(dot(col(R,2), Vector3(0,0,1))>0)
				return R;
		else
				return v3(col(R,0), -col(R,1), -col(R,2));
}

bool ManipController::pushBox()
{

		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->gainParameter[2];

		double handOpenPose[4] ={-1.22173, 1.22173, 1.22173, -1.22173};

		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		Vector3 Po = targetObject->bodyItemObject->body()->link(0)->p;
		Matrix3 Ro = upright(targetObject->bodyItemObject->body()->link(0)->attitude());

		vector<Vector3> pPcr, pNcr, pTcr;
		vector<Matrix3> pRcr;

		for(int i=0; i<2; i++){
			pPcr.push_back( pf->handClusters[i].controlPoints[0] );
			pNcr.push_back( pf->handClusters[i].normal );
			pTcr.push_back( pf->handClusters[i].approachVec[0] );
			pRcr.push_back( v3(pNcr[i], pTcr[i], cross(pNcr[i],pTcr[i]) ) );
		}

		chooseArmToGrasp();
		int nj = arm_g->arm_path->numJoints()-1;

		VectorXd theta(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());
		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		tc->jointSeq.clear();
		tc->pathPlanDOFSeq.clear();
		tc->objectContactStateSeq.clear();
		tc->graspingStateSeq.clear();
		tc->graspingStateSeq2.clear();
		tc->setTrajectoryPlanDOF(graspingHand);
		tc->motionTimeSeq.clear();

		//===== Initial Point
		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		Vector3 oTco(0, 0, -1);
		Vector3 dz(0,0,0.01);
		Matrix3 Rp_, oRco;
		Vector3 Pp_, Pco, oPmargin, oPco, oNco;

		int seqSize = 1;
		double op = 1.0;

		for(int side=0; side<2; side++){
				bool calculated, calculated2;

				if(side==1) op=-1.0;

				oPco << 0.04,  0.027, 0.0; //with offset
				//oPco << 0.04,  0.025, 0.0;//ver1,3
				oNco = Vector3(1,0,0)*op;
				oRco = v3(oNco, oTco, cross(oNco,oTco) );


				for(int i=0; i<2; i++){
						
						calculated = true;
						calculated2 = true;

						oPmargin << 0.01, 0.0, 0.0;
						Pco  = Po + Ro*(oPco+oPmargin)*op + Ro*dz + app_length*z;
						Rp_ = arm_g->arm_path->joint(nj)->calcRfromAttitude(rodrigues(Ro*oTco, m_pi)*Ro*oRco*trans(pRcr[i]));

						//==== Approach Point
						rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
						if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated = false;

						if(calculated){
								//==== Ready to push
								Pco  -= app_length*z;

								rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
								if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;

								//==== Pushing
								for(int j=0; j<3; j++){
										//oPmargin << -0.1/3.0, 0, 0;
										oPmargin << -0.11/3.0, 0,0; //with Offset
										Pco  += Ro*oPmargin*op;

										rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
										if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
				}

								//==== Hand go up
								Pco  += app_length*z - 1.3*Ro*oPmargin*op;

								rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
								if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
						}

						if(calculated && calculated2){
								for(unsigned k=seqSize; k<tc->jointSeq.size(); k++)
										for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
												tc->jointSeq[k](fingers_g[i]->fing_path->joint(j)->jointId) = theta(j + i*fingers_g[0]->fing_path->numJoints());
												tc->jointSeq[k](fingers_g[1-i]->fing_path->joint(j)->jointId) = handOpenPose[j + i*fingers_g[0]->fing_path->numJoints()];
										}


								break;
						}
						else if(!calculated2)
								while ((int)tc->jointSeq.size() > seqSize) tc->jointSeq.pop_back();

				}
				if(calculated && calculated2) break;
				else if(side==1) return false;
		}


		seqSize = tc->jointSeq.size();

		oPco << 0.022,  0.0, 0.0;//with offset
		//oPco << 0.02,  0.0, 0.0;
		oNco = Vector3(0, 1,  0)*op;
		oRco = v3(oNco, oTco, cross(oNco,oTco) );

		for(int i=0; i<2; i++){
			bool calculated = true, calculated2 = true;

			oPmargin << 0.0, 0.01, 0.0;
			Pco  = Po + Ro*(oPco+oPmargin)*op + Ro*dz + app_length*z;
			Rp_ = arm_g->arm_path->joint(nj)->calcRfromAttitude(rodrigues(Ro*oTco, m_pi)*Ro*oRco*trans(pRcr[i]));

			//==== Approach Point
			rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
			if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated = false;

			if(calculated){

				//==== Ready to push
				Pco  -= app_length*z;

				rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;

				//==== Pushing
				for(int j=0; j<2; j++){
					//oPmargin << 0, -0.07/2.0, 0;
					oPmargin << 0, -0.0725/2.0, 0;
					Pco  += Ro*oPmargin*op;

					rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
					if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
				}

				//==== Hand go up
				Pco  += app_length*z - Ro*oPmargin*op;

				rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
			}

			if(calculated && calculated2){
					for(unsigned k=seqSize; k<tc->jointSeq.size(); k++)
							for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
									tc->jointSeq[k](fingers_g[i]->fing_path->joint(j)->jointId) = theta(j + i*fingers_g[0]->fing_path->numJoints());
									tc->jointSeq[k](fingers_g[1-i]->fing_path->joint(j)->jointId) = handOpenPose[j + i*fingers_g[0]->fing_path->numJoints()];
							}

					break;
			}
			else if(!calculated && i==1) return false;
			else if(!calculated2)
					while ((int)tc->jointSeq.size() > seqSize) tc->jointSeq.pop_back();
		}

		seqSize = tc->jointSeq.size();

		oPco << -0.122,  0.0, 0.0;//with offset
		//oPco << -0.11,  0.0, 0.0;
		oNco =Vector3(0, -1,  0)*op;
		oRco = v3(oNco, oTco, cross(oNco,oTco) );

		for(int i=0; i<2; i++){
			bool calculated = true, calculated2 = true;

			oPmargin << 0.0, -0.01, 0.0;
			Pco  = Po + Ro*(oPco+oPmargin)*op + Ro*dz + app_length*z;
			Rp_ = arm_g->arm_path->joint(nj)->calcRfromAttitude(rodrigues(Ro*oTco, m_pi)*Ro*oRco*trans(pRcr[i]));

			//==== Approach Point
			rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
			if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated = false;

			if(calculated){

				//==== Ready to push
				Pco  -= app_length*z;

				rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;

				//==== Pushing
				for(int j=0; j<2; j++){
					//oPmargin << 0, 0.07/2.0, 0;
					oPmargin << 0, 0.0725/2.0, 0;
					Pco  += Ro*oPmargin*op;

					rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
					if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
				}

				//==== Hand go up
				Pco  += app_length*z - Ro*oPmargin*op;

				rb->calcPalmFingParam(robot, Pco, Pco, Rp_, pPcr[i], pNcr[i], pTcr[i], Pp_, theta);
				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_, Rp_, jointSeq, arm_g, fingers_g, mtime[1])) calculated2 = false;
			}

			if(calculated && calculated2){
					for(unsigned k=seqSize; k<tc->jointSeq.size(); k++)
							for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
									tc->jointSeq[k](fingers_g[i]->fing_path->joint(j)->jointId) = theta(j + i*fingers_g[0]->fing_path->numJoints());
									tc->jointSeq[k](fingers_g[1-i]->fing_path->joint(j)->jointId) = handOpenPose[j + i*fingers_g[0]->fing_path->numJoints()];
							}
					break;
			}
			else if(!calculated && i==1) return false;
			else if(!calculated2)
					while ((int)tc->jointSeq.size() > seqSize) tc->jointSeq.pop_back();
		}

		//==Return to initial
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//---------
		bodyItemRobot()->body()->calcForwardKinematics();

		rb->writeFile(robot, tc->jointSeq, tc->motionTimeSeq, false);

		//to remove
		int base[2]={15,19};
		double off = 10.0*m_pi/180.0;
		double offset[3] = {off, 0.0, -off};

		for(unsigned int l=1; l<tc->jointSeq.size(); l++){
				for (int i = 0; i < 2; i++) {
						for (int j = 0; j < 3; j++) {
								int k = base[i] + j;
								tc->jointSeq[l](k) += offset[j];
						}
				}
		}
		
		os << "Success (push box)" << endl;
		return true;
}

void ManipController::calcJointSeq(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp_put, Matrix3& Rp_put){

		calcJointSeqTest(Pp_grasp, Rp_grasp, Pp_put, Rp_put, approachVec);

		return;
}

bool ManipController::doGraspPlanning()
{
#ifdef CALIB_MODE
		rb->Calibration(robot);
#else

		if(robot==HIRO)
				tc->setVisOffsetR();

		if(robot==HIRO && targetObject->bodyItemObject->name()=="W4")
				return movePallet();
		else if(robot==HIRO && targetObject->bodyItemObject->name()=="W10"){
				return pushBox();
		}

		//os << "Gripper Manipulation"  << endl;
		bool success=true;

		Vector3 Pp_put, Pp_grasp; //Palm putting/grasping position
		Matrix3 Rp_put, Rp_grasp;
		VectorXd theta;

		if(strategy==RIGHT_RIGHT || strategy==LEFT_LEFT) {

				if(searchPickAndPlaceMotion(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, false)){

						palmPos = Pp_grasp;
						palmRot = Rp_grasp;

						if(envItem != NULL){
								calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put);

								rb->writeFile(robot, tc->jointSeq, tc->motionTimeSeq);
						}
				}
				else 	success=false;

		} else if(strategy==RIGHT_LEFT || strategy==LEFT_RIGHT){

				if(strategy==RIGHT_LEFT)	graspingHand = RIGHT;
				if(strategy==LEFT_RIGHT)	graspingHand = LEFT;

				Vector3 Pp_tmp_put, Pp_regrasp;
				Matrix3 Rp_tmp_put, Rp_regrasp;
				VectorXd theta_regrasp;

				if(searchPickAndPlaceWithRegrasp(Pp_grasp, Rp_grasp, Pp_tmp_put, Rp_tmp_put, Pp_regrasp, Rp_regrasp, Pp_put, Rp_put, theta, theta_regrasp, false)){

						calcJointSeqTestRegrasp(Pp_grasp, Rp_grasp, Pp_tmp_put, Rp_tmp_put, Pp_regrasp, Rp_regrasp, Pp_put, Rp_put, approachVec, approachVec2);

						rb->writeFile(robot, tc->jointSeq, tc->motionTimeSeq);
				}
				else{
						if(strategy==RIGHT_LEFT){
								strategy=LEFT_LEFT;
								arm_g = tc->arm(1);
								fingers_g[0] = tc->fingers(1,0);
								fingers_g[1] = tc->fingers(1,1);
						}
						if(strategy==LEFT_RIGHT){
								strategy = RIGHT_RIGHT;
								arm_g = tc->arm(0);
								fingers_g[0] = tc->fingers(0,0);
								fingers_g[1] = tc->fingers(0,1);
						}

						if(searchPickAndPlaceMotion(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, false)){
								palmPos = Pp_grasp;
								palmRot = Rp_grasp;

								if(envItem != NULL){
										calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put);

										rb->writeFile(robot, tc->jointSeq, tc->motionTimeSeq);
								}
						}
						else		success = false;
				}
		}


		if(success)
				os << "Success: " << PlacePlanner::instance()->putPos.Index << endl;
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
						bodyItemRobot()->body()->joint(i)->q = tc->jointSeq[0](i);
				bodyItemRobot()->body()->calcForwardKinematics();
				os << "Fail (grasp plan): " << PlacePlanner::instance()->putPos.Index << endl;
				tc->flush();
		}

#endif
		return true;
}

