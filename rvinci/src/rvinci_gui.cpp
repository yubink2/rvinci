#include "rvinci/rvinci_gui.h"
#include <ros/console.h>

namespace rvinci {

void RvinciGui::initialize()
{
    ROS_INFO_STREAM("GUI initialize");
    Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
    overlay_ = overlay_manager.create("RVinciInterface");
    overlay_->setZOrder(500);

    panel_ = static_cast<Ogre::OverlayContainer*>(
	   overlay_manager.createOverlayElement("Panel", "MainPanel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
	panel_->setPosition(10, 10);
	panel_->setDimensions(100, 100);

    textArea_ = static_cast<Ogre::TextAreaOverlayElement*>(
	   overlay_manager.createOverlayElement("TextArea", "TextMessage"));
	textArea_->setMetricsMode(Ogre::GMM_PIXELS);
	textArea_->setPosition(0, 0);
	textArea_->setDimensions(100, 100);
	textArea_->setCharHeight(12);
	// textArea_->setFontName("BlueHighway");
	textArea_->setCaption("sup");
    textArea_->setColourBottom(Ogre::ColourValue(0.3, 0.5, 0.3));
	textArea_->setColourTop(Ogre::ColourValue(0.5, 0.7, 0.5));

    overlay_->add2D(panel_);
    panel_->addChild(textArea_);
    overlay_->show();
}

void RvinciGui::show_overlay() 
{
      if (overlay_) overlay_->show();

    // Ogre::Overlay* left_cursor = Ogre::OverlayManager::getSingleton().getByName("RVinciCursorLeft");
    // if (left_cursor) left_cursor->show();
    // else ROS_INFO_STREAM("show() -- left not found");
    
    // Ogre::Overlay* right_cursor = Ogre::OverlayManager::getSingleton().getByName("RVinciCursorRight");
    // if (right_cursor) right_cursor->show();
    // else ROS_INFO_STREAM("show() -- right not found");
}

void RvinciGui::hide_overlay() {
      if (overlay_) overlay_->hide();

    // Ogre::Overlay* left_cursor = Ogre::OverlayManager::getSingleton().getByName("RVinciCursorLeft");
    // if (left_cursor) left_cursor->hide();
    // else ROS_INFO_STREAM("hide() -- left not found");

    // Ogre::Overlay* right_cursor = Ogre::OverlayManager::getSingleton().getByName("RVinciCursorRight");
    // if (right_cursor) right_cursor->hide();
    // else ROS_INFO_STREAM("hide() -- right not found");
}

} // namespace rvinci
