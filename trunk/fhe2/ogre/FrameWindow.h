#ifndef FRAMEWINDOW_H
#define FRAMEWINDOW_H

#include "Widget.h"

namespace fhe
{
    class FrameWindow : public Widget
    {
        public:
            CEGUI::Window* create( CEGUI::WindowManager* windowManager );
    };
    
    FHE_NODE_DECL(FrameWindow);
}

#endif
