#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_MAIN_PANEL_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_MAIN_PANEL_H_

#include <OgreOverlayManager.h>
#include "icon_button.h"

namespace Ogre {
class PanelOverlayElement;
}

namespace rvinci {

class MainPanel {
public:
    Ogre::OverlayContainer* create();

private:
    Ogre::PanelOverlayElement* main_panel_;
    IconButton button_;

    void createButton(Ogre::OverlayManager& overlay_manager);

};    
    
}  // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_MAIN_PANEL_H_
