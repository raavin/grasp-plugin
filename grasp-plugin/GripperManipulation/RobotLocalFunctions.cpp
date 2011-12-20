// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include <exception>
#include "RobotLocalFunctions.h"
#include <ctime>

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

#define m_pi (3.141592)

void RobotLocalFunctions::calcPalmFingParam(int robot, const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Vector3& pNcr1, const Vector3& pTcr1, Vector3& Pp, VectorXd& theta)
{

		if(robot==HIRO){
				//Vector3 pPcr2( pPcr1 + 0.046*pNcr1 );
				Vector3 pPcr2( pPcr1 + 0.03*pNcr1 );

				double l = 0.0419;
				double S = dot((Rp*pNcr1), (Rp*(pPcr1-pPcr2) - (Pco1-Pco2)) ) / (2.0*l);
				double q = asin(S);
				theta(0) = q;
				theta(1) = -q;
				theta(2) = -q;
				theta(3) = q;

				Matrix3 R=v3(pNcr1, pTcr1, cross(pNcr1,pTcr1));
				Vector3 d1(sin(q), 1-cos(q), 0);
				Pp = Pco1 - Rp*(pPcr1 - l*R*d1);
		}
		else if(robot==PA10){
				Pp = 0.5*(Pco1 + Pco2 - Rp*pPcr1 - Rp*pPcr1 );//?
				theta(0) = -norm2(Pco1-Pco2)/2.0;
				theta(1) = -theta(0);
		}
}

void RobotLocalFunctions::Calibration(int robot) {


		string datafile;
		double t;
		int off;

		if(robot==HIRO || robot==HRP2){

				datafile = "extplugin/graspPlugin/GripperManipulation/PRM/calib_HIRO2.dat";

				ifstream fp(datafile.c_str()) ;
				Vector3 pos, rpy;
				off=1;

				while(!fp.eof()){
						for(int i=0; i<3; i++){
								fp >> t;
								pos(i) = t;
						}
						for(int i=0; i<3; i++){
								fp >> t;
								rpy(i) = t*m_pi/180.0;
						}
						Matrix3 rot=rotFromRpy(rpy);

						if(!PlanBase::instance()->arm()->IK_arm(pos, PlanBase::instance()->arm()->palm->calcRfromAttitude(rot)))
								cout << "IK was not solved" << endl;

						if(!PlanBase::instance()->arm()->checkArmLimit())
								cout << "Joint limit error" << endl;

						double x=0,y=0,z=0;
						if(robot==HIRO){

								cout << "timeToCalib" << off << " = 5.0" << endl;
								cout << "calibPose" << off << " = [" << endl;
								cout << "[ " << PlanBase::instance()->arm()->arm_path->joint(0)->q*180.0/m_pi << ", " << -PlanBase::instance()->arm()->arm_path->joint(0)->q*180.0/m_pi << ", 60], " << endl;
								cout << "[ " << PlanBase::instance()->arm()->arm_path->joint(1)->q*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(2)->q*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(3)->q*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(4)->q*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(5)->q*180.0/m_pi << ", "
									 << PlanBase::instance()->arm()->arm_path->joint(6)->q*180.0/m_pi << "], " << endl;
								cout << "[ 30, 0, -143.2,  0,  0,  0]," << endl;
								cout << "[0, 0, 0, 0]," << endl;
								cout << "[ 0, 0, 0, 0] "<< endl;
								cout << "]" << endl;
								cout << endl;
								
								for(int j=0; j<2; j++)
										for(int i=0; i<PlanBase::instance()->fingers(j)->fing_path->numJoints(); i++)
												PlanBase::instance()->fingers(j)->fing_path->joint(i)->q = 0.0;
								
								PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

								x = -0.030; //-0.0322 + (0.0365*0.5-0.0155);
								y =  0.010; //0.0005 + 0.021*0.5;
								cout << "marker pos = " << Vector3(PlanBase::instance()->fingers(1)->fing_path->joint(0)->p + PlanBase::instance()->fingers(1)->fing_path->joint(0)->attitude()*Vector3(x,y,z)).transpose() << endl;
						}
						else if(robot==HRP2){

								cout << "Joint angles ";
								for(int i=0; i<PlanBase::instance()->bodyItemRobot()->body()->numJoints(); i++)
										cout <<  PlanBase::instance()->bodyItemRobot()->body()->joint(i)->q*180/m_pi << " ";
								cout << endl;

								PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();

								Vector3 Ph = PlanBase::instance()->bodyItemRobot()->body()->link(16)->p;
								Matrix3 Rh = PlanBase::instance()->bodyItemRobot()->body()->link(16)->attitude();
								Vector3 Pw = PlanBase::instance()->arm()->palm->p;
								Matrix3 Rw = PlanBase::instance()->arm()->palm->attitude();
								
								x = 0.045;
								z = -0.14;

								cout << "marker pos " << Vector3(Rh.transpose()*(Pw - Ph + Rw*Vector3(x,y,z))).transpose() << endl;
						}

						PlanBase::instance()->flush();
						usleep(100000);

						off++;
				}
		}
		else {
				datafile = "extplugin/graspPlugin/GripperManipulation/PRM/calib_PA10.dat";
				off=0;

				ifstream fp(datafile.c_str()) ;

				while(!fp.eof()){

						for(int i=off; i<PlanBase::instance()->arm()->arm_path->numJoints(); i++){
								fp >> t;
								//cout << t << " ";
								PlanBase::instance()->arm()->arm_path->joint(i)->q = t*m_pi/180.0;
						}

						//cout << endl;

						PlanBase::instance()->bodyItemRobot()->body()->calcForwardKinematics();
						PlanBase::instance()->flush();

						double x = -0.03;
						double z = 0.056;
						Vector3 offset(x,0,z);
						cout << "marker pos = ";
						cout << Vector3( PlanBase::instance()->arm()->arm_path->joint(6)->p + PlanBase::instance()->arm()->arm_path->joint(6)->attitude()*offset ).transpose() << endl;
						usleep(100000);
						//cout << PlanBase::instance()->arm()->arm_path->joint(6)->attitude() << endl;
				}
		}
}

void RobotLocalFunctions::writeJointSeq(char * jointlogfile, unsigned int filelength, const vector<VectorXd>& jointSeq, vector<double>& motionTimeSeq) {
	time_t theTime;
	time( &theTime );
	tm * now = localtime( &theTime );
	strftime(jointlogfile, filelength, "jointSeq-%Y%m%d%H%M%S.log", now);
	ofstream dout(jointlogfile);

	for(unsigned int i=0; i<jointSeq.size(); i++){
			dout << "-" << endl;
			for (int j = 0; j < 23; j++) {
				dout << "  - " << jointSeq[i](j) << endl;
			}
			dout << "  - " << motionTimeSeq[i] << endl;
	}
}

void RobotLocalFunctions::convertAngles(const VectorXd & seq, vector<double> & angles, double off) {
	const double o = 180.0/m_pi;

	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++) {
		angles[i] = seq(i)*o;
	}
	const int base[] = {15, 19};
	const double offset[3] = {-off, 0.0, off};
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			int k = base[i] + j;
			angles[k] += offset[j];
		}
	}
}

void RobotLocalFunctions::checkAngles(vector<double> & angles) {
	const int asize = (int)angles.size();
	for (int i = 0; i < asize; i++) {
		if (angles[i] > 360.0) {
			throw angles;
		}
	}
}

void RobotLocalFunctions::writeFile(int robot, const vector<VectorXd>& jointSeq, vector<double>& motionTimeSeq, bool hasOffset) {

		if(robot==HIRO){
				double offset = 0.0;
				if(hasOffset) offset = 10.0;

				string datafile = "/opt/grx/HIRONX/script/bodyinfo2.py";

				ofstream fout(datafile.c_str()) ;

				fout << "import bodyinfo" << endl;
				fout << endl;
				fout << "testPatternName = 'Grasp Action' " << endl;
				fout << "testPattern = [" << endl;

				for(unsigned int i=0; i<jointSeq.size(); i++){
						vector<double> angles(23);
						convertAngles(jointSeq[i], angles, offset);
						try {
							checkAngles(angles);
						} catch (std::vector<double> angles) {
							char jointlogfile[256];
							writeJointSeq(jointlogfile, 255, jointSeq, motionTimeSeq);
							cout << "Whrong jointSeq! Check " << jointlogfile << endl;
							return; //throw angles;
						}
						fout << "  [[ [ " << angles[0] << ", " << angles[1] << ", " << angles[2] << "], [";

						for(unsigned int j=0; j<5; j++)
								fout << angles[j+3] << ", ";

						fout << angles[8] << "], [";

						for(int j=0; j<5; j++)
								fout << angles[j+9] << ", ";

						fout << angles[14] << "],[],[]], [";

						for(int j=0; j<3; j++)
								fout << angles[15+j] << ", ";

						fout << angles[18] << "], [";

						for(int j=0; j<3; j++)
								fout << angles[19+j] << ", ";

						fout << angles[22] << "], " << motionTimeSeq[i] << "]," << endl;
				}

				fout << "]" << endl;
		}
		else{
				//=== Used in VVVInterface
				DIR *pDir;
				pDir = opendir("extplugin/graspPlugin/VVVInterface");
				double o = 180.0/m_pi;

				if(pDir != NULL){

						ofstream gout("extplugin/graspPlugin/VVVInterface/data/grasp.mat");

						gout <<"6 7" << endl;
						for(unsigned int i=1; i<jointSeq.size(); i++)
								gout << jointSeq[i](0)*o << " "<< jointSeq[i](1)*o << " "<< jointSeq[i](2)*o << " "<< jointSeq[i](3)*o << " "<< jointSeq[i](4)*o+180.0 << " "<< -jointSeq[i](5)*o << " "<< jointSeq[i](6)*o << endl;

				}

				//system("./auto_scp2.sh");
				int res = system("cp extplugin/graspPlugin/VVVInterface/data/grasp.mat ~/freeformdemo/shells2");
				if (res) {
					cout << "Copying grasp.mat to shells2 failed! (" << res << ")" << endl;
				}
		}

		return;
}
