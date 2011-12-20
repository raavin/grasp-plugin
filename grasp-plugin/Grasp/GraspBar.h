/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include "exportdef.h"


namespace cnoid {

    class MessageView;
	
}
    
namespace grasp {

        class EXCADE_API GraspBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static GraspBar* instance();

            virtual ~GraspBar();

            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
                return sigBodyItemSelectionChanged_;
            }

            cnoid::SignalProxy< boost::signal<void(cnoid::BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
                return sigCurrentBodyItemChanged_;
            }

            const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems() {
                return selectedBodyItems_;
            }

            cnoid::BodyItem* currentBodyItem() {
                return currentBodyItem_.get();
            }

            bool makeSingleSelection(cnoid::BodyItemPtr bodyItem);

          protected:

            virtual bool storeState(cnoid::Archive& archive);
            virtual bool restoreState(const cnoid::Archive& archive);

          private:

            GraspBar();

            cnoid::MessageView& mes;
			std::ostream& os;
	//	  GraspController* gc;
//			TrajectoryPlanner* trajectoryPlanner_;

			cnoid::BodyItemPtr currentBodyItem_;
            cnoid::ItemList<cnoid::BodyItem> selectedBodyItems_;
            cnoid::ItemList<cnoid::BodyItem> targetBodyItems;

            boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

            void onItemSelectionChanged(const cnoid::ItemList<cnoid::BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();
            void onObjectButtonClicked();
            void onRobotButtonClicked();
			void onEnvironButtonClicked();
			void onRemoveEnvButtonClicked();
            void onGraspButtonClicked();
            void onPlaceButtonClicked();
            void onPickAndPlaceButtonClicked();
            void onStopButtonClicked();
			void onTrajectoryPlanButtonClicked();
            void onArmButtonClicked();
            void onCloseButtonClicked();
			void onSaveGPButtonClicked();
			void onSelectGraspPattern()	;
			void onDisplayGRCPositionButtonClicked();
 //           void onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy);
//            void onZmpCmButtonClicked();
//            void setZmp(BodyItem::ZmpPosition position);
        };
}

#endif
