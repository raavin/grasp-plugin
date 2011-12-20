// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "ClusterParameter.h"

//#define DEBUG

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

ParameterFileData* ParameterFileData::instance() {
		static ParameterFileData* instance = new ParameterFileData();
		return instance;
}

void ParameterFileData::calcIDPair(int intention)
{
		idPairs.clear();

		for(unsigned int id0=0; id0<objClusters.size(); id0++){
				
				if(objClusters[id0].intention != intention) continue;
				
				for(unsigned int j=0; j<objClusters[id0].idPair.size(); j++){
						
						int id1=-1;
						for(unsigned int k=0; k<objClusters.size(); k++){
								
								if(objClusters[k].id != objClusters[id0].idPair[j]) continue;
								if(objClusters[k].intention != objClusters[id0].intention) continue;
								id1=k;
								break;
						}
						
						if(id1 == -1) continue;
						
						vector<int> id;
						id.push_back(id0);
						id.push_back(id1);
						idPairs.push_back(id);
				}
		}		
}

void ParameterFileData::readObjClusterParameters()
{
		graspPostureCandidate.clear();
		for(int i=0; i<8; i++)
				graspPostureCandidate.push_back(i);

		struct stat st;
		
		string objectpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + ".prm";
		string objyamlpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + ".yaml";
		objClusters.clear();
		if(stat(objyamlpath.c_str(), &st)==0)
				readObjYamlFile(PlanBase::instance()->targetObject->bodyItemObject, objClusters);
		else if(stat(objectpath.c_str(), &st)==0)				
				readPRMFile(objectpath.c_str(), objClusters);

		string gripperpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->bodyItemRobot()->name() + ".prm";
		handClusters.clear();
		if(stat(gripperpath.c_str(), &st)==0)
				readPRMFile(gripperpath.c_str(), handClusters);
		else
				readObjYamlFile(PlanBase::instance()->bodyItemRobot(), handClusters);

}

void ParameterFileData::readEnvClusterParameters()
{
		struct stat st;

		string objectpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + ".prm";
		string objenvpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + "env.prm";
		string objyamlpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + ".yaml";

		objEnvClusters.clear();
		if(stat(objyamlpath.c_str(), &st)==0)
				readEnvYamlFile(PlanBase::instance()->targetObject->bodyItemObject, objEnvClusters);
		else if(stat(objenvpath.c_str(), &st)==0)
				readPRMFile(objenvpath.c_str(), objEnvClusters);
		else if(stat(objectpath.c_str(), &st)==0)
				readPRMFile(objectpath.c_str(), objEnvClusters);

		envClusters.clear();
		int i=0;
		for(list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it != PlanBase::instance()->bodyItemEnv.end(); ++it){
				
				string envpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + (*it)->body()->name() + ".prm";
				string ymlpath = "extplugin/graspPlugin/GripperManipulation/PRM/" + (*it)->body()->name() + ".yaml";

				if(stat(ymlpath.c_str(), &st)==0)
						readEnvYamlFile(*it, envClusters);
				else if(stat(envpath.c_str(), &st)==0)
						readPRMFile(envpath.c_str(), envClusters);

				for(unsigned int j=i; j<envClusters.size(); j++){
						envClusters[j].normal   = (*it)->body()->link(0)->attitude()*envClusters[j].normal;
						envClusters[j].tangent  = (*it)->body()->link(0)->attitude()*envClusters[j].tangent;
						envClusters[j].bbCenter = (*it)->body()->link(0)->p + (*it)->body()->link(0)->attitude()*envClusters[j].bbCenter;
						for(unsigned int k=0; k<envClusters[j].convexhullPoints.size(); k++)
								envClusters[j].convexhullPoints[k] = (*it)->body()->link(0)->p + (*it)->body()->link(0)->attitude()*envClusters[j].convexhullPoints[k];
						for(unsigned int k=0; k<envClusters[j].boundaryPointList.size(); k++)
								for(unsigned int l=0; l<envClusters[j].boundaryPointList[k].size(); l++)
										envClusters[j].boundaryPointList[k][l] = (*it)->body()->link(0)->p + (*it)->body()->link(0)->attitude()*envClusters[j].boundaryPointList[k][l];
				}
				i += envClusters.size();
		}
}

void ClusterParameter::deleteCluster(){

		controlPoints.clear();
		approachVec.clear();
		convexhullPoints.clear();
		boundaryPointList.clear();
		idPair.clear();

		normal <<0,0,0;
		tangent <<0,0,0;
		area = 0.0;

		isPuttingCluster = false;
}

void ClusterParameter::printCluster(){

		cout << "id " << id << endl;
		cout << "idPair ";
		for(unsigned int i=0; i<idPair.size(); i++)
				cout << idPair[i] << " ";
		cout << endl;
		cout << "Control Points: " << endl;
		for(unsigned int i=0; i<controlPoints.size(); i++)
				cout << controlPoints[i].transpose() << endl;
		cout << "Approach Vectors: " << endl;
		for(unsigned int i=0; i<approachVec.size(); i++)
				cout << approachVec[i].transpose() << endl;
		cout << "BBedge ";
		cout << bbEdge.transpose() << endl;
}

void ParameterFileData::readPRMFile(const char fname[], vector<ClusterParameter>& clusterSet)
{
		ifstream fp;
		double x;
		Vector3 p;
		char Jname[256];
		
		bool write=false;
		ClusterParameter cluster;

		cluster.isPuttingCluster = false;
		
		fp.open(fname);
		
		while(GetString(fp,Jname)){
				//cout << "Jname:" << Jname << endl;
				if(strcmp(Jname,"Plane") == 0 || strcmp(Jname,"Gripper") == 0){
						
						while( GetVRMLdouble(fp,x) )
								;
						
						if(write)
							clusterSet.push_back(cluster);
						cluster.deleteCluster();
						
						cluster.id = (int)x;
						write = true;
				}
				else if(strcmp(Jname,"id") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.id = (int)x;
				}
				else if(strcmp(Jname,"pair") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.idPair.push_back((int)x);
				}
				else if(strcmp(Jname,"area") == 0){
						
						while( GetVRMLdouble(fp,x) )
								cluster.area = x;
				}
				else if(strcmp(Jname,"intention") == 0){
						
						while( GetVRMLdouble(fp,x) )
								cluster.intention = (int)x;
				}
				else if(strcmp(Jname, "outer_normal") == 0){
						
						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.normal(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "tangent_vector") == 0){
						
						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.tangent(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "approach_vector") == 0){
						
						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.approachVec.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "control_points") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.controlPoints.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "convexhull") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.convexhullPoints.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "boundary") == 0){

						int i=0;
						vector<Vector3> points;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) points.push_back(p);
								i++;
						}
						cluster.boundaryPointList.push_back(points);

				}
				else if(strcmp(Jname, "bounding_box_edge") == 0){
						
						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.bbEdge(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "bounding_box_center") == 0){
						
						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.bbCenter(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "grasp_posture") == 0){
						graspPostureCandidate.clear();
						while( GetVRMLdouble(fp,x) )
								graspPostureCandidate.push_back((int)x);
				}
				else if(strcmp(Jname, "gain_values") == 0){
						gainParameter.clear();
						while( GetVRMLdouble(fp,x) )
								gainParameter.push_back(x);
				}
				else if(strcmp(Jname, "motion_time") == 0){
						motionTime.clear();
						while( GetVRMLdouble(fp,x) )
								motionTime.push_back(x);
				}
				else if(strcmp(Jname, "is_putting_cluster") == 0){
						while( GetVRMLdouble(fp,x) )
								if((int)x==1) cluster.isPuttingCluster=true;
				}
		}

		clusterSet.push_back(cluster);
		
		fp.close();
		
		return;
}

void ParameterFileData::readObjYamlFile(BodyItemPtr item, vector<ClusterParameter>& clusterSet)
{

	ClusterParameter cluster;
	Vector3 p;
	cluster.isPuttingCluster = false;

	if( item->body()->info()->find("Cluster")->type() == YAML_SEQUENCE){
			const YamlSequence& glist = *(*item->body()->info())["Cluster"].toSequence();
			for(int i=0;i<glist.size();i++){
					
					const YamlMapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {
							
							ClusterParameter cluster;
							cluster.isPuttingCluster = false;
							
							if( gSettings.find("id")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["id"].toSequence();
									cluster.id = list[0].toInt();
							}
							
							if( gSettings.find("pair")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["pair"].toSequence();
									for(int i=0;i<list.size();i++){
											cluster.idPair.push_back(list[i].toInt());
									}
							}
							
							if( gSettings.find("area")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["area"].toSequence();
									cluster.area = list[0].toDouble();
							}
							
							if( gSettings.find("intention")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["intention"].toSequence();
									cluster.intention = list[0].toInt();
							}
							
							if( gSettings.find("outer_normal")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["outer_normal"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.normal(i) = list[i].toDouble();
							}
							
							if( gSettings.find("tangent_vector")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["tangent_vector"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.tangent(i) = list[i].toDouble();
							}
							
							if( gSettings.find("approach_vector")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["approach_vector"].toSequence();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.approachVec.push_back(p);
									}
							}
							
							if( gSettings.find("control_points")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["control_points"].toSequence();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.controlPoints.push_back(p);
									}
							}
							
							if( gSettings.find("bounding_box_edge")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["bounding_box_edge"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.bbEdge(i) = list[i].toDouble();
							}
							
							if( gSettings.find("bounding_box_center")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["bounding_box_center"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.bbCenter(i) = list[i].toDouble();
							}
							
							clusterSet.push_back(cluster);
					}
			}
	}
	
	if( item->body()->info()->find("GripperManipulationParameter")->type() == YAML_SEQUENCE){
			const YamlSequence& glist = *(*item->body()->info())["GripperManipulationParameter"].toSequence();
			for(int i=0;i<glist.size();i++){

					const YamlMapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {

							if( gSettings.find("grasp_posture")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["grasp_posture"].toSequence();
									for(int i=0;i<list.size();i++)
											graspPostureCandidate.push_back( list[i].toInt() );
							}
							
							if( gSettings.find("gain_values")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["gain_values"].toSequence();
									for(int i=0;i<list.size();i++)
											gainParameter.push_back( list[i].toDouble() );
							}
							
							if( gSettings.find("motion_time")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["motion_time"].toSequence();
									for(int i=0;i<list.size();i++)
											motionTime.push_back( list[i].toDouble() );
							}
					}
			}
	}
	

		
	return;
}


void ParameterFileData::readEnvYamlFile(BodyItemPtr item, vector<ClusterParameter>& clusterSet)
{

	ClusterParameter cluster;
	Vector3 p;
	cluster.isPuttingCluster = false;

	if( item->body()->info()->find("EnvCluster")->type() == YAML_SEQUENCE){
			const YamlSequence& glist = *(*item->body()->info())["EnvCluster"].toSequence();
			for(int i=0;i<glist.size();i++){
					
					const YamlMapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {
							
							ClusterParameter cluster;
							cluster.isPuttingCluster = false;
							


							if( gSettings.find("id")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["id"].toSequence();
									cluster.id = list[0].toInt();
							}

							if( gSettings.find("area")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["area"].toSequence();
									cluster.area = list[0].toDouble();
							}

							if( gSettings.find("outer_normal")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["outer_normal"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.normal(i) = list[i].toDouble();
							}
							
							if( gSettings.find("tangent_vector")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["tangent_vector"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.tangent(i) = list[i].toDouble();
							}

							
							if( gSettings.find("cenvexhull")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["convexhull"].toSequence();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.convexhullPoints.push_back(p);
									}
							}
							
							if( gSettings.find("boundary")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["boundary"].toSequence();
									vector<Vector3> pList;
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													pList.push_back(p);
									}
									cluster.boundaryPointList.push_back(pList);
							}
							
							if( gSettings.find("bounding_box_edge")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["bounding_box_edge"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.bbEdge(i) = list[i].toDouble();
							}
							
							if( gSettings.find("bounding_box_center")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["bounding_box_center"].toSequence();
									for(int i=0;i<list.size();i++)
											cluster.bbCenter(i) = list[i].toDouble();
							}

							if( gSettings.find("is_putting_cluster")->type() == YAML_SEQUENCE ){ 
									const YamlSequence& list = *gSettings["is_putting_cluster"].toSequence();
									cluster.isPuttingCluster = false;
									if( list[0].toInt() == 1)
											cluster.isPuttingCluster = true;
							}
							clusterSet.push_back(cluster);
					}
			}
	}
		
	return;
}




