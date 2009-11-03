#ifndef BUTTON_H
#define BUTTON_H

#include "Widget.h"

namespace gge
{
    namespace Graphics
    {
        
        class Button : public Widget
        {
            public:
                Button();
                
                void on_attach();
                
                void set_text( Var val );
                
                bool onClick( const CEGUI::EventArgs& evt );
        };

    }
}

#endif
