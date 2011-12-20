/**
   @author Shin'ichiro Nakaoka
*/

#ifndef cnoid_ROBOTICS_GRASP_BAR_H_INCLUDED
#define cnoid_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

//#include <cnoidPlugins/Grasp/TrajectoryPlanner.h>

using namespace cnoid;


//    class MessageView;
    
    namespace grasp {

        class EXCADE_API GraspRtcBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static GraspRtcBar* instance();

            virtual ~GraspRtcBar();

          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            GraspRtcBar();

            MessageView& mes;
			std::ostream& os;

            void onStartButtonClicked();
            void onStartButtonClicked2();
            void onStopButtonClicked();
        };
    }

#endif
