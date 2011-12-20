/*! @file
  @author Tokuo Tsuji
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/

#include <osgManipulator/Projector>
#include <osg/Geometry>

#include <cnoid/SceneBody>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/SceneBodyManager>	/* modified by qtconv.rb 0th rule*/

#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

class GeodeFinder : public osg::NodeVisitor {
   public:

   // Constructor - sets the traversal mode to TRAVERSE_ALL_CHILDREN
   // and Visitor type to NODE_VISITOR
   GeodeFinder();

   // The 'apply' method for 'node' type instances.
   // See if a className() call to searchNode returns "Geode."
   // If so, add this node to our list.
   void apply(osg::Node &searchNode);

   // Return a pointer to the first node in the list
   // with a matching name
   osg::Geode* getFirst();

   // return a the list of nodes we found
   std::vector<osg::Geode*> getNodeList();

   private:
   // List of nodes with names that match the searchForName string
   std::vector<osg::Geode*> foundGeodes;
};

GeodeFinder::GeodeFinder ()
   : NodeVisitor (osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

void GeodeFinder::apply (osg::Node &searchNode) {
   if (! strcmp (searchNode.className(), "Geode")) {
      foundGeodes.push_back ((osg::Geode*) &searchNode);
   }
   traverse (searchNode);
}

osg::Geode* GeodeFinder::getFirst () {
   if (foundGeodes.size() > 0)
      return foundGeodes.at(0);
   else
      return NULL;
}

std::vector<osg::Geode*> GeodeFinder::getNodeList() {
   return foundGeodes;
}



class EditModelSceneBody : public cnoid::SceneBody {
public:
	EditModelSceneBody(cnoid::BodyItemPtr bodyItem) : SceneBody(bodyItem) { isDragging=false;}

protected:

	virtual ~EditModelSceneBody() {  };

//	    virtual void onAttachedToScene();
//	    virtual void onDetachedFromScene();

	virtual bool onKeyPressEvent(const SceneViewEvent& event);
//    virtual bool onKeyReleaseEvent(const SceneViewEvent& event);
	    virtual bool onButtonPressEvent(const SceneViewEvent& event);
	    virtual bool onButtonReleaseEvent(const SceneViewEvent& event);
//	    virtual bool onDoubleClickEvent(const SceneViewEvent& event);
	    virtual bool onPointerMoveEvent(const SceneViewEvent& event);
//	    virtual void onPointerLeaveEvent(const SceneViewEvent& event);
//	    virtual void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
//	    virtual void onSceneModeChanged();
//	    virtual bool onUndoRequest();
//	    virtual bool onRedoRequest();

private:

		bool printVertexTriangleData(const SceneViewEvent& event);

        cnoid::Link* targetLink;
        osg::ref_ptr<osgManipulator::Projector> projector;
        osgManipulator::PointerInfo pointerInfo;
		cnoid::Vector3 pressPos;
//        osg::ref_ptr<osg::Node>  shapeNode;

		std::vector<osg::Geode*> gList;
		std::vector<osg::Vec3Array*> verticesList;

		bool isDragging;

};
typedef osg::ref_ptr<EditModelSceneBody> EditModelSceneBodyPtr;


bool EditModelSceneBody::printVertexTriangleData(const SceneViewEvent& event){


	osg::Node* shapeNode = getPointedShapeNode();
	GeodeFinder geodeFinder;
	shapeNode->accept (geodeFinder);
	gList  = geodeFinder.getNodeList() ;

	int cnt=0;
	for(int i=0;i< gList.size(); i++){
		for(int j=0;j<gList[i]->getNumDrawables();j++){
			osg::Geometry* geometry=gList[i]->getDrawable(j)->asGeometry();
			if(geometry){
				osg::Vec3Array* vertices = (osg::Vec3Array*)geometry->getVertexArray();
				cout << endl;
				cout << endl;
				cout << "point [ " << endl;
				for(int k=0;k<vertices->size();k++){
					cout << (*vertices)[k].x() << " " <<  (*vertices)[k].y() <<" "<<(*vertices)[k].z() << endl;
				}
				cout << "]  " << endl;
				cout << "} " << endl;
				cout << "coordIndex [" << endl;

				osg::IntArray& vertexIndices = *((osg::IntArray*)geometry->getVertexIndices()) ;
				osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));

				int index = 0;

				for(int k=0;k<lengths.size();k++){
					for(int l=0; l<lengths[k];l++){
						cout << vertexIndices[index++] <<" ";
					}
					cout << "-1" <<endl;
				}
				cout << "]  " << endl;
				cout << "} " << endl;
			}
		}
	}


}


bool EditModelSceneBody::onKeyPressEvent(const SceneViewEvent& event) {

    bool handled = true;

    int key = event.key();
    key = toupper(key);

    switch(key){
    case 'P':
		printVertexTriangleData(event);
		return handled;
    }


	return cnoid::SceneBody::onKeyPressEvent(event);
}


bool EditModelSceneBody::onButtonPressEvent(const SceneViewEvent& event) {
    bool handled = false;

	if ( (event.modKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) &&
         (event.button() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) ) { /* modified by qtconv.rb 3rd rule*/
		targetLink = getPointedSceneLink();
		pressPos = cnoid::Vector3(event.point().x(), event.point().y(),
				event.point().z());

		Vector3 normal = cnoid::Vector3(event.normal().x(), event.normal().y(),
				event.normal().z());

		cout << pressPos << endl;
		cout << normal << endl;
		//			return cnoid::SceneBody::onButtonPressEvent(event);
		//			return true;

		osg::Vec3 eye, center, up;
		event.camera()->getViewMatrixAsLookAt(eye, center, up);
		projector = new osgManipulator::PlaneProjector(osg::Plane(eye - center,
				event.point()));
		pointerInfo.reset();
		pointerInfo.setCamera(event.camera());

		osg::Node* shapeNode = getPointedShapeNode();

		//			osg::MatrixList matList=  shapeNode->getWorldMatrices	( 0 );
		//			for(int i=0;i<4;i++) for(int j=0;j<4;j++) cout << matList[0](i,j) << " ";
		//			cout << endl;

		GeodeFinder geodeFinder;
		shapeNode->accept(geodeFinder);
		gList = geodeFinder.getNodeList();
		for (int i = 0; i < gList.size(); i++) {
			for (int j = 0; j < gList[i]->getNumDrawables(); j++) {
				//					osg::MatrixList matList= gList[0]->getDrawable(0)->asGeometry()->getWorldMatrices(0) ;
				//					cout << matList.size() << endl;

				osg::Vec3Array
						* vertices =
								new osg::Vec3Array(
										*((osg::Vec3Array*) gList[i]->getDrawable(
												j)->asGeometry()->getVertexArray()),
										osg::CopyOp::DEEP_COPY_ALL);
				verticesList.push_back(vertices);
			}
		}

		isDragging = true;

		return true;
	}

	return cnoid::SceneBody::onButtonPressEvent(event);
}


bool EditModelSceneBody::onButtonReleaseEvent(const SceneViewEvent& event) {


	if(isDragging){
		gList.clear();
		for(int i=0;i<verticesList.size();i++){
			((osg::Vec3Array*)verticesList[i])->clear();
		}
		verticesList.clear();
	}
	isDragging = false;

	return cnoid::SceneBody::onButtonReleaseEvent(event);
}

bool EditModelSceneBody::onPointerMoveEvent(const SceneViewEvent& event)
{
	Vector3 unit[3];
	unit[0] = Vector3( 1,0,0);
	unit[1] = Vector3( 0,1,0);
	unit[2] = Vector3( 0,0,1);
	if( (event.modKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) && isDragging ){

//		Matrix3 rot ( trans( targetLink->attitude() )*targetLink->R );
		Matrix3 rot ( targetLink->attitude()  );

		osg::Vec3d pos0;
		pointerInfo.setMousePosition(event.x(), event.y());
		if(projector->project(pointerInfo, pos0)){
			const cnoid::Vector3 dir(  rot.transpose() * ( Vector3(pos0.x(), pos0.y(), pos0.z()) - pressPos));
			double max=0;
			Vector3 iDir;
			for(int i=0;i<3;i++){
				if( fabs( dir.dot(unit[i]) ) > max) {
					iDir = unit[i];
					max =  fabs( dir.dot(unit[i]) );
				}
			}
			double dist = dir.dot( iDir)  /  iDir.dot(  rot.transpose() * (pressPos - targetLink->p)) ;
			Vector3 ampl  (iDir*dist + Vector3(1,1,1) );
			int cnt=0;
			for(int i=0;i< gList.size(); i++){
				for(int j=0;j<gList[i]->getNumDrawables();j++){
					osg::Geometry* geometry=gList[i]->getDrawable(j)->asGeometry();
					if(geometry){

						Matrix3 rot;
						rot << 1,0,0,0,1,0,0,0,1;
						osg::MatrixList worldMatrices = geometry->getWorldMatrices();
						for(osg::MatrixList::iterator itr = worldMatrices.begin(); itr != worldMatrices.end(); ++itr) {
							osg::Matrix& matrix = *itr;
							Matrix3 rot2;
							rot2 << matrix(0,0),matrix(0,1),matrix(0,2),matrix(1,0),matrix(1,1),matrix(1,2),matrix(2,0),matrix(2,1),matrix(2,2);
							rot = rot2*rot;
						}
						const cnoid::Vector3 dir(  ( rot ) * ( Vector3(pos0.x(), pos0.y(), pos0.z()) - pressPos));
						double max=0;
						Vector3 iDir;
						for(int k=0;k<3;k++){
							if( fabs( dir.dot(unit[k]) ) > max) {
								iDir = unit[k];
								max =  fabs( dir.dot(unit[k]) );
							}
						}
						double dist = dir.dot(iDir)  /  iDir.dot(( rot ) * (pressPos - targetLink->p)) ;
						Vector3 ampl  (iDir*dist + Vector3(1,1,1) );

						osg::Vec3Array* vertices = (osg::Vec3Array*)gList[i]->getDrawable(j)->asGeometry()->getVertexArray();
						for(int k=0;k<vertices->size();k++){
							(*vertices)[k] = osg::Vec3 ( (*verticesList[cnt])[k].x()*ampl[0],(*verticesList[cnt])[k].y()*ampl[1],(*verticesList[cnt])[k].z()*ampl[2]);
						}
						geometry->setVertexArray(vertices);
					}
					cnt++;
				}
			}
		}
		requestRedraw();
		return false;
	}
	return cnoid::SceneBody::onPointerMoveEvent(event);
}



//cnoid::SceneBody* createEditModel(cnoid::BodyItem* item);
//EditModelSceneBody* editModelSceneBody;


cnoid::SceneBody* createEditModel(cnoid::BodyItem* item) {
	return new EditModelSceneBody(item);;
}


class EditModelPlugin : public cnoid::Plugin {
private:

public:

//	static EditModelSceneBody* editModelSceneBody;



	EditModelPlugin() : Plugin("EditModel") {
		depend("Body");
	}

	~EditModelPlugin() { }



	virtual bool initialize() {
		cnoid::SceneBodyManager* manager = cnoid::SceneBodyManager::instance();
		manage(manager->addSceneBodyFactory(createEditModel));
		return true;
	}

};


CNOID_IMPLEMENT_PLUGIN_ENTRY(EditModelPlugin);
//EXCADE_IMPLEMENT_PLUGIN_ENTRY(EditModelPlugin);
