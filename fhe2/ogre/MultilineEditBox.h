#ifndef MULTILINE_MultilineEditBox_H
#define MULTILINE_MultilineEditBox_H

#include "Widget.h"

namespace fhe
{
    class MultilineEditBox : public Widget
    {
        private:
            CEGUI::MultiLineEditbox* m_multilineEditBox;
            
        public:
            MultilineEditBox();
            
            CEGUI::Window* create( CEGUI::WindowManager* windowManager );
            
            void set_text( Var val );
            Var get_text();
    };
    
    FHE_NODE_DECL(MultilineEditBox);
}

#endif
