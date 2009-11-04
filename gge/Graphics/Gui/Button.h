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
                
                Var on_attach( const Var& val );
                
                Var set_text( const Var& val );
                
                bool onClick( const CEGUI::EventArgs& evt );
        };

    }
}

#endif
