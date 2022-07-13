#ifndef RVINCI_INCLUDE_RVINCI_ICON_BUTTON_H_
#define RVINCI_INCLUDE_RVINCI_ICON_BUTTON_H_

#include <OgreOverlayManager.h>
#include <OgrePrerequisites.h>
#include <OgreBlendMode.h>

namespace Ogre {
class PanelOverlayElement;
}

namespace rvinci {

class IconButton {
public:
    IconButton& create(Ogre::OverlayManager& manager, const std::string& name);
    IconButton& atPosition(Ogre::Real left, Ogre::Real top);
    IconButton& withDimensions(Ogre::Real width, Ogre::Real height);

private:
    Ogre::PanelOverlayElement* panel_ = nullptr;
    // bool is_enabled_ = true;
};

} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_ICON_BUTTON_H_
