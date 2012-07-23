// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "PlacePlanner.h"

#define m_pi 3.141592

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::GripperManipulation;

PlacePlanner::PlacePlanner()  : 	os (MessageView::mainInstance()->cout() ) {

}

PlacePlanner::~PlacePlanner() {
}

PlacePlanner* PlacePlanner::instance() {
		static PlacePlanner* instance = new PlacePlanner();
		return instance;
}

bool includeClusterBBox(const Vector3& pos, ClusterParameter& cluster){ //included in environment BBox

		double eps = 1.0e-6;

		if(fabs(dot(cluster.tangent, pos - cluster.bbCenter)) > 0.5*cluster.bbEdge(0)+eps
		   ||	fabs(dot(unit(cross(cluster.normal, cluster.tangent)), pos - cluster.bbCenter)) > 0.5*cluster.bbEdge(1)+eps
		   ||	fabs(dot(cluster.normal, pos - cluster.bbCenter)) > 0.5*cluster.bbEdge(2)+eps)
				return false;
		return true;
}

bool includeClusterBBox2(const Vector3& pos, ClusterParameter& cluster){ //included in 2d projection of enviromnent BBox

		double eps = 1.0e-6;

		if(fabs(dot(cluster.tangent, pos - cluster.bbCenter)) > 0.5*cluster.bbEdge(0)+eps
		   ||	fabs(dot(unit(cross(cluster.normal, cluster.tangent)), pos - cluster.bbCenter)) > 0.5*cluster.bbEdge(1)+eps)
				return false;
		return true;
}


void PlacePlanner::calcObjPosFaceFace(const Vector3& pos, ClusterParameter& env, ClusterParameter& obj, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put)
{
		cout << "FaceFace" << endl;

		Matrix3 E_e = v3(env.tangent, cross(env.normal, env.tangent), env.normal);

		Matrix3 oE_o, oE_o2;
		if(((env.bbEdge[0] < env.bbEdge[1]) && (obj.bbEdge[0] < obj.bbEdge[1])) || ((env.bbEdge[0] >= env.bbEdge[1]) && (obj.bbEdge[0] >= obj.bbEdge[1]))){
				oE_o  = v3( obj.tangent, -unit(cross(obj.normal, obj.tangent)), -obj.normal);
				oE_o2 = v3(-obj.tangent,  unit(cross(obj.normal, obj.tangent)), -obj.normal);
		}
		else if(((env.bbEdge[0] < env.bbEdge[1]) && (obj.bbEdge[0] >= obj.bbEdge[1])) || ((env.bbEdge[0] >= env.bbEdge[1]) && (obj.bbEdge[0] < obj.bbEdge[1]))){
				oE_o  = v3( unit(cross(obj.normal, obj.tangent)),  obj.tangent, -obj.normal);
				oE_o2 = v3(-unit(cross(obj.normal, obj.tangent)), -obj.tangent, -obj.normal);
		}

		Vector3 P_ec = pos;
		if((env.bbEdge(0) < 2*obj.bbEdge(0) && env.bbEdge(1) < 2*obj.bbEdge(1)) || (env.bbEdge(0) < 2*obj.bbEdge(1) && env.bbEdge(1) < 2*obj.bbEdge(0)))
				P_ec = env.bbCenter + 0.5*env.normal*env.bbEdge(2);

		Ro_put.push_back( E_e*oE_o.transpose() );
		Ro_put.push_back( E_e*oE_o2.transpose() );

		int j = Ro_put.size() -1;
		Po_put.push_back( P_ec - Ro_put[j-1]*(obj.bbCenter + 0.5*obj.normal*obj.bbEdge(2)) );
		Po_put.push_back( P_ec - Ro_put[j  ]*(obj.bbCenter + 0.5*obj.normal*obj.bbEdge(2)) );

}


void PlacePlanner::calcPutPos(Vector3& pressPos, const Matrix3& pressOri, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put){

		cp = new CollisionPair();
		pf = new ParameterFileData();

		pf->readEnvClusterParameters();
		cp->setCollisionObj();

		int ide=-1;
		for(unsigned int i=0; i<pf->envClusters.size(); i++)
				if(includeClusterBBox(pressPos, pf->envClusters[i])){
						ide = i;
						break;
				}

		vector<int> idList;
		for(unsigned int i=0; i<pf->objEnvClusters.size(); i++)
				if(pf->objEnvClusters[i].isPuttingCluster)
						idList.push_back(i);


		if(idList.size()==0 || ide==-1 ){
				cout << "Setting error in PRM file " << idList.size() << "/" << ide << endl;
				return;
		}

		Vector3 envEdge = sort(pf->envClusters[ide].bbEdge);

		Po_put.clear();
		Ro_put.clear();

		for(unsigned int i=0; i<idList.size(); i++){

				int ido = idList[i];

				Vector3 objEdge = sort( pf->objEnvClusters[ido].bbEdge );

				if(objEdge(1) < envEdge(1) && objEdge(2) < envEdge(2) ){ //FaceFace
						calcObjPosFaceFace(pressPos, pf->envClusters[ide], pf->objEnvClusters[ido], Po_put, Ro_put);
						//for(unsigned int j=0; j<Po_put.size(); j++)
						//		cout << isGravityStable(pf->envClusters[ide], pf->objEnvClusters[ido], Po_put[j], Ro_put[j]) << endl;
				}
				//else  //FaceEdge
						//calcObjPosEdgeFace(pressPos, pf->envClusters[ide], pf->objEnvClusters[ido], Po_put, Ro_put);
		}

		if(Po_put.size()==0){
				cout << "cannot put object " << endl; //onto " << envItem->body()->name() << endl;
				Po_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p);
				Ro_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude());
				return;
		}

		//cout << "Current object " << PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p.transpose() << endl;
		//cout << PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude() << endl;
		cout << "Target object " << endl; for(unsigned int j=0; j<Po_put.size(); j++){ cout << Po_put[j].transpose() << endl; cout << Ro_put[j] << endl;}

		return;
}
void generateRotMatFromVec( double vx, double vy, double vz, Matrix3 &rotMat )
{
    // Get our direction vector (the Z vector component of the matrix)
    // and make sure it's normalized into a unit vector
    double vnorm = sqrt( vx*vx+vy*vy+vz*vz );
    double zvx, zvy, zvz;
    zvx = vx/vnorm;
    zvy = vy/vnorm;
    zvz = vz/vnorm;

    // Build the Y vector of the matrix (handle the degenerate case
    // in the way that 3DS does) -- This is not the TRUE vector, only
    // a reference vector.
    double yvx, yvy, yvz;
    if ( !zvx && !zvz ) {
        yvx = -zvy;
        yvy = 0.;
        yvz = 0.;
    } else {
        yvx = 0.;
        yvy = 1.;
        yvz = 0.;
    }

    // Build the X axis vector based on the two existing vectors
    double xvx, xvy, xvz;
    xvx = zvy*yvz - zvz*yvy;
    xvy = zvz*yvx - zvx*yvz;
    xvz = zvx*yvy - zvy*yvx;

    vnorm = sqrt( xvx*xvx + xvy*xvy + xvz*xvz );
    xvx /= vnorm;
    xvy /= vnorm;
    xvz /= vnorm;

    // Correct the Y reference vector
    yvx = xvy*zvz - xvz*zvy;
    yvy = xvz*zvx - xvx*zvz;
    yvz = xvx*zvy - xvy*zvx;
    vnorm = sqrt( yvx*yvx + yvy*yvy + yvz*yvz );
    yvx = -yvx/vnorm;
    yvy = -yvy/vnorm;
    yvz = -yvz/vnorm;

    // Generate rotation matrix without roll included
    rotMat( 0, 0 ) = xvx;   rotMat( 0, 1 ) = yvx;   rotMat( 0, 2 ) = zvx;
    rotMat( 1, 0 ) = xvy;   rotMat( 1, 1 ) = yvy;   rotMat( 1, 2 ) = zvy;
    rotMat( 2, 0 ) = xvz;   rotMat( 2, 1 ) = yvz;   rotMat( 2, 2 ) = zvz;
}


bool PlacePlanner::findTargetTriangle( cnoid::Link *targetLink, cnoid::Vector3 &pressPos, cnoid::BodyItemPtr pBody)
{

		const double precision = 0.001;
		
		if( targetLink == NULL ) return false;
		
		// compute the pressPos 3D coordinates in the object coordinates system
		objPressPos = targetLink->R.transpose()*(pressPos - targetLink->p);
		PlanBase::instance()->objPressPos = objPressPos;
		double x = objPressPos(0);
		double y = objPressPos(1);
		double z = objPressPos(2);
		
		cout << " Press Pos " << objPressPos.transpose() << endl;
		cout << " Press Pos " << pressPos.transpose() << endl;

		objPressName = pBody->body()->name();
		PlanBase::instance()->objPressName = objPressName;

		vector<int> triangleList;
		vector<double> distanceList;
		double nnx, nny, nnz, mindist=0.;
		Vector3 nnp;
		int mindisti=-1;
		
		for( int i=0; i<targetLink->coldetModel->getNumTriangles(); ++i ) {
				int t1, t2, t3;
				float tx, ty, tz;
				targetLink->coldetModel->getTriangle( i, t1, t2, t3 );
				targetLink->coldetModel->getVertex( t1, tx, ty, tz );
				Vector3 p1(tx,ty,tz);
				targetLink->coldetModel->getVertex( t2, tx, ty, tz );
				Vector3 p2(tx,ty,tz);
				targetLink->coldetModel->getVertex( t3, tx, ty, tz );
				Vector3 p3(tx,ty,tz);

				// check the bounding box
				double minx=p1(0), maxx=p1(0),  miny=p1(1), maxy=p1(1),  minz=p1(2), maxz=p1(2);
				if( p2(0)<minx ) minx = p2(0);
				if( p3(0)<minx ) minx = p3(0);
				if( p2(1)<miny ) miny = p2(1);
				if( p3(1)<miny ) miny = p3(1);
				if( p2(2)<minz ) minz = p2(2);
				if( p3(2)<minz ) minz = p3(2);
				if( p2(0)>maxx ) maxx = p2(0);
				if( p3(0)>maxx ) maxx = p3(0);
				if( p2(1)>maxy ) maxy = p2(1);
				if( p3(1)>maxy ) maxy = p3(1);
				if( p2(2)>maxz ) maxz = p2(2);
				if( p3(2)>maxz ) maxz = p3(2);
				
				if( x < minx-precision ) continue;
				if( x > maxx+precision ) continue;
				if( y < miny-precision ) continue;
				if( y > maxy+precision ) continue;
				if( z < minz-precision ) continue;
				if( z > maxz+precision ) continue;
				
				// triangle normal vector
				Vector3 v1 = p2 - p1;
				Vector3 v2 = p3 - p1;
				Vector3 np = unit(cross(v1, v2));
				
				// check the distance between the target point and the plane
				double planeD = -dot(np, p1);
				double ppdist = fabs( dot(np, objPressPos) + planeD); //nx*x + ny*y + nz*z + planeD );
				
				// the point should be on the surface of the object -> the distance should be close to 0
				// we do not test if the point is inside the triangle !! Thus we may get a list of triangles!
				if( ppdist < precision ) {
						distanceList.push_back( ppdist );
						triangleList.push_back( i );
						
						if( ppdist<mindist || mindisti<0 ) {
								mindist = ppdist; 
								mindisti = i;
								
								// must rotate the normal vector to match the world coordinates!
								nnp = targetLink->R*np;
						}
				}
		}
		
		// construct and display the target location as an arrow
		static bool firstRun = true;
		static cnoid::BodyItem *localArrowBI = new cnoid::BodyItem;
		if( firstRun ) {
#ifdef CNOID_10_11
				boost::filesystem::path robotfullpath( pBody->modelFilePath() );
#else
				boost::filesystem::path robotfullpath( pBody->lastAccessedFilePath() );
#endif
				std::string bodyItemRobotPath = boost::filesystem::path( robotfullpath.branch_path() ).string();
				localArrowBI->loadModelFile( bodyItemRobotPath + "/../project/arrowhrp.wrl" );
		}
		
		if( localArrowBI!=NULL ) {
				localArrowBI->body()->rootLink()->p = pressPos;
				cnoid::Matrix3 rotMat;
				generateRotMatFromVec( nnp(0), nnp(1), nnp(2), rotMat );
				cnoid::Vector3 normVec( nnp );
				localArrowBI->body()->rootLink()->R = rotMat;
				
				localArrowBI->notifyKinematicStateChange();
				
				// send the results to the controller 
				//clusterNormal.clear();
				//clusterNormal.push_back(-trans(pBody->body()->link(0)->attitude())*normVec);
				//counter=0;
		}
		
		static SceneBody *localArrowSB = new SceneBody( localArrowBI );
		if( firstRun ) SceneView::mainInstance()->addSceneObject( localArrowSB );
		
		MessageView::mainInstance()->flush();
		SceneView::mainInstance()->requestRedraw();
		
		firstRun = false;
		
		//  SceneView::mainInstance()->removeSceneObject( localArrowSB );
		return true;
}

