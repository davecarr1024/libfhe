#include "FrameWindow.h"

namespace fhe
{
    FHE_ASPECT(FrameWindow);
 
    CEGUI::Window* FrameWindow::create( CEGUI::WindowManager* windowManager )
    {
        return windowManager->createWindow("TaharezLook/FrameWindow",getPath());
    }
}
