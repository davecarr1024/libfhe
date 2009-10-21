#include "Widget.h"
#include "CEGUIUtil.h"

namespace fhe
{
    FHE_NODE_IMPL(Widget);
    
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
            m_window = create(windowManager);
        }
        
        if ( m_window )
        {
            parentWindow->addChildWindow( m_window );
        }
        
        defaultVar("pos",Vec2(0,0));
        defaultVar("size",Vec2(1,1));
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
        for ( NodePtr node = getParent(); node; node = node->getParent() )
        {
            if ( node->hasFunc<CEGUI::Window*,void>("getWindow") )
            {
                return node->call<CEGUI::Window*>("getWindow");
            }
        }
        return 0;
    }
    
    CEGUI::WindowManager* Widget::getWindowManager()
    {
        for ( NodePtr node = getParent(); node; node = node->getParent() )
        {
            if ( node->hasFunc<CEGUI::WindowManager*,void>("getWindowManager") )
            {
                return node->call<CEGUI::WindowManager*>("getWindowManager");
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
