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
    
    void Button::set_text( std::string text )
    {
        if ( m_button )
        {
            m_button->setText(text);
        }
    }
    
    std::string Button::get_text()
    {
        return m_button ? m_button->getText().c_str() : "";
    }
    
    bool Button::onClick( const CEGUI::EventArgs& evt )
    {
        log("clicked!");
    }
    
}
