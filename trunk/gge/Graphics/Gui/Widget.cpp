#include "Widget.h"
#include "Graphics/CEGUIUtil.h"

namespace gge
{
    namespace Graphics
    {
        
        GGE_ASPECT(Widget);
        
        Widget::Widget()
        {
            addFunc("on_attach",&Widget::on_attach,this);
            addFunc("on_detach",&Widget::on_detach,this);
            addFunc("set_widgetParent",&Widget::set_widgetParent,this);
            addFunc("set_widget",&Widget::set_widget,this);
            addFunc("set_pos",&Widget::set_pos,this);
            addFunc("get_pos",&Widget::get_pos,this);
            addFunc("set_size",&Widget::set_size,this);
            addFunc("get_size",&Widget::get_size,this);
        }
        
        Widget::~Widget()
        {
    /*        on_detach();
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            if ( widget )
            {
                delete widget;
                getEntity()->setVar<CEGUI::Window*>("widget",0);
            }*/
        }
        
        Var Widget::on_attach( const Var& arg )
        {
            getEntity()->defaultVar<std::string>("widgetParent","Window");
            getEntity()->defaultVar<Vec2>("pos",Vec2(0,0));
            getEntity()->defaultVar<Vec2>("size",Vec2(1,1));
            return Var();
        }
        
        Var Widget::on_detach( const Var& arg )
        {
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            if ( widget )
            {
                CEGUI::Window* parentWidget = widget->getParent();
                if ( parentWidget )
                {
                    parentWidget->removeChildWindow(widget);
                }
            }
            return Var();
        }
        
        Var Widget::set_widgetParent( const Var& val )
        {
            EntityPtr parentEntity = getEntity()->getApp()->getEntity(val.get<std::string>("Window"));
            assert(parentEntity);
            CEGUI::Window* parentWidget = parentEntity->getVar<CEGUI::Window*>("widget",0);
            assert(parentWidget);
            
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            if ( widget )
            {
                parentWidget->addChildWindow(widget);
            }
            return Var();
        }
        
        Var Widget::set_widget( const Var& val )
        {
            on_attach(Var());
            return Var();
        }
        
        Var Widget::set_pos( const Var& val )
        {
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            if ( widget && val.is<Vec2>() )
            {
                widget->setPosition(CEGUIUtil::fromVec2(val.get<Vec2>()));
            }
            return Var();
        }
        
        Var Widget::get_pos( const Var& arg )
        {
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            return widget ? Var::build<Vec2>(CEGUIUtil::toVec2(widget->getPosition())) : Var();
        }
        
        Var Widget::set_size( const Var& val )
        {
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            if ( widget && val.is<Vec2>() )
            {
                widget->setSize(CEGUIUtil::fromVec2(val.get<Vec2>()));
            }
            return Var();
        }
        
        Var Widget::get_size( const Var& arg )
        {
            CEGUI::Window* widget = getEntity()->getVar<CEGUI::Window*>("widget",0);
            return widget ? Var::build<Vec2>(CEGUIUtil::toVec2(widget->getSize())) : Var();
        }
        
        CEGUI::WindowManager* Widget::getWindowManager()
        {
            EntityPtr window = getEntity()->getApp()->getEntity("Window");
            assert(window);
            CEGUI::WindowManager* wm = window->getVar<CEGUI::WindowManager*>("windowManager",0);
            assert(wm);
            return wm;
        }
    }
}
