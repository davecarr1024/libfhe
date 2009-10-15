#ifndef BUTTON_H
#define BUTTON_H

#include "Widget.h"

namespace fhe
{
    class Button : public Widget
    {
        private:
            CEGUI::PushButton* m_button;
            
        public:
            Button();
            
            CEGUI::Window* create( CEGUI::WindowManager* windowManager );
            
            void set_text( Var val );
            Var get_text();
            
            bool onClick( const CEGUI::EventArgs& evt );
    };
}

#endif
