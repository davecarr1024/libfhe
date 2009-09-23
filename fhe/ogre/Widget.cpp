#include "Widget.h"
#include "CEGUIUtil.h"

namespace fhe
{
    NODE_IMPL(Widget);
    
    Widget::Widget( const std::string& name, const std::string& type ) :
        Node(name,type),
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
                return node->callFunc<CEGUI::Window*>("getWindow");
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
                return node->callFunc<CEGUI::WindowManager*>("getWindowManager");
            }
        }
        return 0;
    }
    
    CEGUI::Window* Widget::create( CEGUI::WindowManager* windowManager )
    {
        return 0;
    }
    
    void Widget::set_pos( Vec2 pos )
    {
        if ( m_window )
        {
            m_window->setPosition( CEGUIUtil::Vec2ToCEGUIVec2( pos ) );
        }
    }
    
    Vec2 Widget::get_pos()
    {
        return m_window ? CEGUIUtil::CEGUIVec2ToVec2( m_window->getPosition() ) : Vec2();
    }
    
    void Widget::set_size( Vec2 size )
    {
        if ( m_window )
        {
            m_window->setSize( CEGUIUtil::Vec2ToCEGUIVec2( size ) );
        }
    }
    
    Vec2 Widget::get_size()
    {
        return m_window ? CEGUIUtil::CEGUIVec2ToVec2( m_window->getSize() ) : Vec2(1,1);
    }
}
