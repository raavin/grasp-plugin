/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
//#include "exportdef.h"

#include "TrajectoryPlanner.h"

#include "exportdef.h"

using namespace cnoid;

namespace cnoid {

    class MessageView;
	
}
    
    namespace grasp {

        class EXCADE_API TrajectoryBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static TrajectoryBar* instance();

            virtual ~TrajectoryBar();


          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            TrajectoryBar();

            MessageView& mes;
		std::ostream& os;
	//	  GraspController* gc;
			TrajectoryPlanner* trajectoryPlanner_;

//			BodyItemPtr currentBodyItem_;
  //          ItemList<BodyItem> selectedBodyItems_;
    //        ItemList<BodyItem> targetBodyItems;

     //       boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

            void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();
	    void onTrajectoryPlanButtonClicked();
	    void onResetButtonClicked();
	    void onSetStartMotionStateButtonClicked();
	    void onSetEndMotionStateButtonClicked();
	  
        };
    }

#endif
