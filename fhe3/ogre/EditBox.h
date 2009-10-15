#ifndef EDITBOX_H
#define EDITBOX_H

#include "Widget.h"

namespace fhe
{
    class EditBox : public Widget
    {
        private:
            CEGUI::Editbox* m_editBox;
            
        public:
            EditBox();
            
            CEGUI::Window* create( CEGUI::WindowManager* windowManager );
            
            void set_text( Var val );
            Var get_text();

            bool onAccepted( const CEGUI::EventArgs& evt );
    };
}

#endif
