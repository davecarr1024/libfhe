#ifndef WIDGET_H
#define WIDGET_H

#include <gge/Aspect.h>

#include <CEGUI.h>

namespace gge
{
    namespace Graphics
    {
        
        class Widget : public Aspect
        {
            protected:
                CEGUI::WindowManager* getWindowManager();
            
            public:
                Widget();
                ~Widget();
                
                void on_attach();
                void on_detach();
                
                void set_widgetParent( Var val );
                
                void set_widget( Var val );
                
                void set_pos( Var val );
                Var get_pos();
                
                void set_size( Var val );
                Var get_size();
        };

    }
}

#endif
