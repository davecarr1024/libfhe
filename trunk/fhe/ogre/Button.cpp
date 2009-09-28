#include "Button.h"

namespace fhe
{
    NODE_IMPL(Button);
    
    Button::Button( const std::string& name, const std::string& type ) :
        Widget(name,type),
        m_button(0)
    {
        addFunc("set_text",&Button::set_text,this);
        addFunc("get_text",&Button::get_text,this);
    }
    
    CEGUI::Window* Button::create( CEGUI::WindowManager* windowManager )
    {
        m_button = (CEGUI::PushButton*)windowManager->createWindow("TaharezLook/Button",getPath());
        m_button->subscribeEvent(CEGUI::PushButton::EventClicked,
            CEGUI::Event::Subscriber(&Button::onClick,this));
        return m_button;
    }
    
    void Button::set_text( Var val )
    {
        if ( m_button && val.is<std::string>() )
        {
            m_button->setText(val.get<std::string>());
        }
    }
    
    Var Button::get_text()
    {
        Var val;
        if ( m_button )
        {
            val.set<std::string>(m_button->getText().c_str());
        }
        return val;
    }
    
    bool Button::onClick( const CEGUI::EventArgs& evt )
    {
        publish<std::string>("buttonClicked",getPath());
    }
    
}
