#ifndef RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
#define RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_

#include <ros/ros.h>

#include <OgreOverlayManager.h>
#include <OgreOverlay.h>
#include <OgreOverlayContainer.h>
#include <OgreTextAreaOverlayElement.h>
#include "rvinci/main_panel.h"

namespace Ogre {
class Overlay;
}

namespace rvinci {

class RvinciGui {
public:
  void initialize();
  void show_overlay();
  void hide_overlay();

private:
  Ogre::Overlay* overlay_ = nullptr;
  Ogre::OverlayContainer* panel_;
  Ogre::TextAreaOverlayElement* textArea_;
//   MainPanel panel_{};

};

} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
