#ifndef WIDGET_H
#define WIDGET_H

#include <fhe/Aspect.h>
#include <fhe/math/Vec2.h>

#include <CEGUI.h>

namespace fhe
{
    
    class Widget : public Aspect
    {
        private:
            CEGUI::Window* m_window;
            
        public:
            Widget();
            
            void on_attach();
            void on_detach();
            
            virtual CEGUI::Window* create( CEGUI::WindowManager* windowManager );
            
            CEGUI::Window* getWindow();
            CEGUI::Window* getParentWindow();
            CEGUI::WindowManager* getWindowManager();
            
            void set_pos( Var val );
            Var get_pos();
            
            void set_size( Var val );
            Var get_size();
    };
}

#endif
