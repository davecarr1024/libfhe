#include "Widget.h"
#include "CEGUIUtil.h"

#include <fhe/Entity.h>

namespace fhe
{
    FHE_ASPECT(Widget);
    
    Widget::Widget() :
        m_window(0)
    {
        addFunc("on_attach",&Widget::on_attach,this);
        addFunc("on_detach",&Widget::on_detach,this);
        addFunc("getWindow",&Widget::getWindow,this);
        addFunc("set_pos",&Widget::set_pos,this);
        addFunc("get_pos",&Widget::get_pos,this);
        addFunc("set_size",&Widget::set_size,this);
        addFunc("get_size",&Widget::get_size,this);
    }
    
    void Widget::on_attach()
    {
        on_detach();
        
        log("on_attach %p",m_window);
        
        CEGUI::Window* parentWindow = getParentWindow();
        if ( !parentWindow )
        {
            error("couldn't find parent window");
        }
        
        if ( !m_window )
        {
            CEGUI::WindowManager* windowManager = getWindowManager();
            if ( !windowManager )
            {
                error("couldn't get windowManager");
            }
            log("create");
            m_window = create(windowManager);
        }
        
        if ( m_window )
        {
            log("add");
            parentWindow->addChildWindow( m_window );
        }
        
        getEntity()->defaultVar("pos",Vec2(0,0));
        getEntity()->defaultVar("size",Vec2(1,1));
    }
    
    void Widget::on_detach()
    {
        if ( m_window )
        {
            CEGUI::Window* parent = m_window->getParent();
            if ( parent )
            {
                parent->removeChildWindow( m_window );
            }
        }
    }
    
    CEGUI::Window* Widget::getWindow()
    {
        return m_window;
    }
    
    CEGUI::Window* Widget::getParentWindow()
    {
        for ( EntityPtr entity = getParentEntity(); entity; entity = entity->getParent() )
        {
            if ( entity->hasFunc<CEGUI::Window*,void>("getWindow") )
            {
                return entity->call<CEGUI::Window*>("getWindow");
            }
        }
        return 0;
    }
    
    CEGUI::WindowManager* Widget::getWindowManager()
    {
        for ( EntityPtr entity = getParentEntity(); entity; entity = entity->getParent() )
        {
            if ( entity->hasFunc<CEGUI::WindowManager*,void>("getWindowManager") )
            {
                return entity->call<CEGUI::WindowManager*>("getWindowManager");
            }
        }
        return 0;
    }
    
    CEGUI::Window* Widget::create( CEGUI::WindowManager* windowManager )
    {
        return windowManager->createWindow("DefaultGUISheet",getPath());
    }
    
    void Widget::set_pos( Var val )
    {
        if ( m_window && val.is<Vec2>() )
        {
            m_window->setPosition( CEGUIUtil::Vec2ToCEGUIVec2( val.get<Vec2>() ) );
        }
        else
        {
            log("warning: discarding pos");
        }
    }
    
    Var Widget::get_pos()
    {
        Var val;
        if ( m_window )
        {
            val.set<Vec2>(CEGUIUtil::CEGUIVec2ToVec2( m_window->getPosition() ));
        }
        return val;
    }
    
    void Widget::set_size( Var val )
    {
        if ( m_window && val.is<Vec2>() )
        {
            m_window->setSize( CEGUIUtil::Vec2ToCEGUIVec2( val.get<Vec2>() ) );
        }
        else
        {
            log("warning: discarding size %p %d",m_window,val.is<Vec2>());
        }
    }
    
    Var Widget::get_size()
    {
        Var val;
        if ( m_window )
        {
            val.set<Vec2>(CEGUIUtil::CEGUIVec2ToVec2( m_window->getSize() ));
        }
        return val;
    }
}
