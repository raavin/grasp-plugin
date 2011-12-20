/**
   @author Tokuo Tsuji
*/

#ifndef GRASP_IMAGEWIDGETPLUGIN_H_INCLUDED
#define GRASP_IMAGEWIDGETPLUGIN_H_INCLUDED

#include <cnoid/View>

namespace grasp {


    class ImageWidgetView : public cnoid::View
    {
    public:
        static bool initialize(cnoid::ExtensionManager* ext);

        ImageWidgetView(std::string* name=NULL);
        ~ImageWidgetView();
    
	bool imageUpdate;

//        virtual bool storeState(Archive& archive);
  //      virtual bool restoreState(const Archive& archive);

    protected:
        QWidget* screen;
	    
//        virtual bool event(QEvent* event);
        virtual void resizeEvent(QResizeEvent* event);
        virtual void paintEvent(QPaintEvent* event);
  //      virtual QPaintEngine* paintEngine () const;
        virtual void onActivated();
        virtual void onDeactivated();

    };
}

#endif
