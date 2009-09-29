#include "Button.h"

namespace fhe
{
    FHE_NODE_IMPL(Button);
    
    Button::Button() :
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
        Var var;
        if ( m_button )
        {
            var.set<std::string>(m_button->getText().c_str());
        }
        return var;
    }
    
    bool Button::onClick( const CEGUI::EventArgs& evt )
    {
        VarMap args;
        args.setVar("path",getPath());
        getRoot()->publish("buttonClicked",args);
    }
}
