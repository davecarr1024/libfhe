#include "FrameWindow.h"

namespace fhe
{
    FHE_NODE_IMPL(FrameWindow);
 
    CEGUI::Window* FrameWindow::create( CEGUI::WindowManager* windowManager )
    {
        return windowManager->createWindow("TaharezLook/FrameWindow",getPath());
    }
}
