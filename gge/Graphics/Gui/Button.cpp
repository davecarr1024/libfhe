#include "Button.h"

namespace gge
{
    namespace Graphics
    {
        
        GGE_ASPECT(Button);
        
        Button::Button()
        {
            addFunc("on_attach",&Button::on_attach,this);
            addFunc("set_text",&Button::set_text,this);
        }
        
        Var Button::on_attach( const Var& arg )
        {
            CEGUI::PushButton* button = static_cast<CEGUI::PushButton*>(
                getWindowManager()->createWindow("TaharezLook/Button",getPath()));
            button->subscribeEvent(CEGUI::PushButton::EventClicked,
                CEGUI::Event::Subscriber(&Button::onClick,this));
            getEntity()->setVar<CEGUI::Window*>("widget",button);
            
            getEntity()->defaultVar<std::string>("text","");
            
            Widget::on_attach(Var());
            return Var();
        }
        
        Var Button::set_text( const Var& val )
        {
            CEGUI::PushButton* button = static_cast<CEGUI::PushButton*>(getEntity()->getVar<CEGUI::Window*>("widget",0));
            if ( button && val.is<std::string>() )
            {
                button->setText(val.get<std::string>());
            }
            return Var();
        }
        
        bool Button::onClick( const CEGUI::EventArgs& evt )
        {
            getEntity()->getApp()->publish("buttonClicked",Var::build<std::string>(getEntity()->getName()));
            
            return true;
        }

    }
}
