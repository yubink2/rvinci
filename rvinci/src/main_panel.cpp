#include "rvinci/main_panel.h"

#include <ros/ros.h>

#include <OgreMaterialManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreViewport.h>

#include "rvinci/main_panel.h"

namespace rvinci {

namespace {
// da Vinci viewer has a large unusable area at the bottom, so scrubber has to
// be fairly high up
constexpr Ogre::Real kPanelTop = 0.6;
constexpr Ogre::Real kPanelHeight = 0.05;
constexpr Ogre::Real kPanelPadding = 0.05;
constexpr Ogre::Real kScrubberBarHeight = 0.1;
constexpr Ogre::Real kScrubberDotSize = 0.5;
constexpr Ogre::Real kButtonWidth = 0.05;
constexpr Ogre::Real kButtonMargin = 0.01;
constexpr Ogre::Real kButtonPadding = 0.0075;

constexpr Ogre::Real kPanelWidth = 1. - 2. * kPanelPadding;
constexpr Ogre::Real kAbsoluteScrubberBarHeight =
    kPanelHeight * kScrubberBarHeight;
constexpr Ogre::Real kAbsoluteScrubberDotSize = kPanelHeight * kScrubberDotSize;
constexpr Ogre::Real kScrubberBarPadding =
    kButtonWidth + 2 * kButtonMargin + kAbsoluteScrubberDotSize / 2.;
constexpr Ogre::Real kScrubberBarWidth = 1 - 2. * kScrubberBarPadding;
constexpr Ogre::Real kAbsoluteScrubberBarWidth =
    kScrubberBarWidth * kPanelWidth;
constexpr Ogre::Real kScrubberDotZero =
    kScrubberBarPadding - kAbsoluteScrubberDotSize / 2.;

} // namespace

// Ogre::OverlayContainer* MainPanel::create() {
//     Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

//     main_panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
//         overlay_manager.createOverlayElement("Panel", "RvinciPanel"));
//     main_panel_->setPosition(0, 10);
//     main_panel_->setDimensions(400, 400);
//     main_panel_->setMaterialName("Template/PartialTransparent");

//     createButton(overlay_manager);

//     return main_panel_;
// }

// void MainPanel::createButton(Ogre::OverlayManager& overlay_manager) {
//     main_panel_->addChild(
//         button_.create(overlay_manager, "SomeButton")
//             .atPosition(kButtonPadding + kButtonMargin,
//                         kButtonPadding - kButtonWidth / 2.)
//             .withDimensions(kButtonWidth - 2 * kButtonPadding,
//                             kButtonWidth - 2 * kButtonPadding)
//             .withMaterial("Template/PlayIcon")
//             .withClickTopic("~rvinci_play_pause")
//             .disabled()
//             .done());
// }

} // namespace rvinci
