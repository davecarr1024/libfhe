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
                
                Var on_attach( const Var& arg );
                Var on_detach( const Var& arg );
                
                Var set_widgetParent( const Var& val );
                
                Var set_widget( const Var& val );
                
                Var set_pos( const Var& val );
                Var get_pos( const Var& arg );
                
                Var set_size( const Var& val );
                Var get_size( const Var& arg );
        };

    }
}

#endif
