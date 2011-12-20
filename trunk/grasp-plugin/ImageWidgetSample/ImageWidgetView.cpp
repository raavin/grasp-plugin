/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageWidgetView.h"
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <cnoid/Sleep>
#include <QEvent>
#include <boost/bind.hpp>


#include <QApplication> 
#include <QGraphicsView> 
#include <QGraphicsScene> 
#include <QGraphicsItem> 
#include <QPrinter> 
#include <QImage> 

#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

namespace {
    const bool TRACE_FUNCTIONS = false;
    const bool TRACE_FUNCTIONS2 = false;

    Action* aspectRatioCheck = 0;
    Action* orgSizeCheck = 0;
}

bool ImageWidgetView::initialize(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        MenuManager& mm = ext->menuManager();
        mm.setPath(tr("/Options/Image View"));
        aspectRatioCheck = mm.addCheckItem(tr("Keep aspect ratio"));
        aspectRatioCheck->setChecked(true);
        orgSizeCheck = mm.addCheckItem(tr("Keep original size"));
        orgSizeCheck->setChecked(true);
    }
}


ImageWidgetView::ImageWidgetView(string* name)
{
	if(name){
		setName(name->c_str());
	}
	else{
		setName("ImageView");
	}
	imageUpdate = true;
}


ImageWidgetView::~ImageWidgetView()
{
	return;
}


void ImageWidgetView::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "ImageWidgetView::resizeEvent()" << endl;
    }
    
}


void ImageWidgetView::paintEvent(QPaintEvent* event)
{
	if(TRACE_FUNCTIONS){
		cout << "ImageWidgetView::paintEvent()" << endl;
	}
	QPainter painter (this);
    
	//if(!imageUpdate) return;

	QImage image(640,480,QImage::Format_RGB888);
	//image.load("2.png");
	//painter.drawImage(QPoint(0, 0), image);
    
	cout <<  image.width() <<  image.height() << endl;
    
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.fillRect(image.rect(), QColor(255, 0, 255, 0));
	painter.setBrush(Qt::blue);
	painter.setPen(Qt::NoPen);
	painter.translate(100,100);
	painter.drawEllipse(0, 0, image.width()/2, image.height()/2);
	painter.end();

	imageUpdate=false;
} 



void ImageWidgetView::onActivated()
{
	if(TRACE_FUNCTIONS){
		cout << "ImageWidgetView::onActivated()" << endl;
	}
	imageUpdate=true;

}


void ImageWidgetView::onDeactivated()
{
    if(TRACE_FUNCTIONS){
        cout << "ImageWidgetView::onDeactivated()" << endl;
    }
}


/*
bool ImageWidgetView::storeState(Archive& archive)
{
//    archive.write("keepAspectRatio", aspectRatioCheck->isChecked());
 //   archive.write("keepOriginalSize", orgSizeCheck->isChecked());
    return true;
}


bool ImageWidgetView::restoreState(const Archive& archive)
{
//    aspectRatioCheck->setChecked(archive.get("keepAspectRatio", aspectRatioCheck->isChecked()));
 //   orgSizeCheck->setChecked(archive.get("keepOriginalSize", orgSizeCheck->isChecked()));
    return true;
}
*/

