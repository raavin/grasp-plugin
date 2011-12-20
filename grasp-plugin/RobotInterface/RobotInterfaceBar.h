/**
   @author Shin'ichiro Nakaoka
*/

#ifndef EXCADE_ROBOTICS_VVV_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_VVV_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

#include "RobotInterface.h"

using namespace cnoid;

    
    namespace grasp {

        class EXCADE_API RobotInterfaceBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static RobotInterfaceBar* instance();
            virtual ~RobotInterfaceBar();


          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

	    RobotInterfaceBar();
	    
	    MessageView& mes;
	    std::ostream& os;
	    //   VVVInterface* vvvInterface_;
	    
	    boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
	    boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

	    void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
	    void onBodyItemDetachedFromRoot();
	    void onReadFromFileButtonClicked();
	    void onRecognitionButtonClicked();
	    void onMultiButtonClicked();
	    
	    void onCaptureButtonClicked();
	    void onJointCalibButtonClicked();
	    void onSrvOnButtonClicked();
	    void onSrvOffButtonClicked();
	    void onHomeButtonClicked();
	    void onMoveButtonClicked();
		  
		  };
    }

#endif
