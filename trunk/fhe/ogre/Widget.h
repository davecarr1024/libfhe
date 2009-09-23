#ifndef WIDGET_H
#define WIDGET_H

#include <fhe/Node.h>
#include <fhe/math/Vec2.h>

#include <CEGUI.h>

namespace fhe
{
    
    class Widget : public Node
    {
        private:
            CEGUI::Window* m_window;
            
        public:
            Widget( const std::string& name, const std::string& type );
            
            void on_attach();
            void on_detach();
            
            virtual CEGUI::Window* create( CEGUI::WindowManager* windowManager );
            
            CEGUI::Window* getWindow();
            CEGUI::Window* getParentWindow();
            CEGUI::WindowManager* getWindowManager();
            
            void set_pos( Vec2 pos );
            Vec2 get_pos();
            
            void set_size( Vec2 size );
            Vec2 get_size();
    };

    NODE_DECL(Widget);
}

#endif
