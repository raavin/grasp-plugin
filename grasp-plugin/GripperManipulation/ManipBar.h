/**
   c) Kensuke Harada (AIST)
*/

#ifndef MANIPBAR_H
#define MANIPBAR_H

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

using namespace cnoid;

namespace cnoid {
    class MessageView;
}


namespace grasp {

    namespace GripperManipulation {

        class EXCADE_API ManipBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static ManipBar* instance();

            virtual ~ManipBar();
	    /*
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
                return sigBodyItemSelectionChanged_;
            }

            SignalProxy< boost::signal<void(BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
                return sigCurrentBodyItemChanged_;
            }

            const ItemList<BodyItem>& selectedBodyItems() {
                return selectedBodyItems_;
            }

            BodyItem* currentBodyItem() {
                return currentBodyItem_.get();
            }

            bool makeSingleSelection(BodyItemPtr bodyItem);
	    */
          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            ManipBar();

            MessageView& mes;
	    std::ostream& os;

	    //BodyItemPtr currentBodyItem_;
            //ItemList<BodyItem> selectedBodyItems_;
            //ItemList<BodyItem> targetBodyItems;

            //boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;

            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

            //void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            //void onBodyItemDetachedFro            void onFirstButtonClicked();mRoot();
            void onAllButtonClicked();
            void onFirstButtonClicked();
            void onSecondButtonClicked();
	    void onThirdButtonClicked();
	    void onFourButtonClicked();


            void onStartButtonClicked();
            //void onStopButtonClicked();
	    //void onTrajectoryPlanButtonClicked();
            //void onArmButtonClicked();
            void onSymmetricCopyButtonClicked(int direction, bool doMirrorCopy);
            void onZmpCmButtonClicked();
            //void setZmp(BodyItem::ZmpPosition position);
        };
    }
}

#endif
