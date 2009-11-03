#include "Button.h"

namespace gge
{
    
    GGE_ASPECT(Button);
    
    Button::Button()
    {
        addFunc("on_attach",&Button::on_attach,this);
        addFunc("set_text",&Button::set_text,this);
    }
    
    void Button::on_attach()
    {
        CEGUI::PushButton* button = static_cast<CEGUI::PushButton*>(
            getWindowManager()->createWindow("TaharezLook/Button",getPath()));
        button->subscribeEvent(CEGUI::PushButton::EventClicked,
            CEGUI::Event::Subscriber(&Button::onClick,this));
        getEntity()->setVar<CEGUI::Window*>("widget",button);
        
        getEntity()->defaultVar<std::string>("text","");
        
        Widget::on_attach();
    }
    
    void Button::set_text( Var val )
    {
        CEGUI::PushButton* button = static_cast<CEGUI::PushButton*>(getEntity()->getVar<CEGUI::Window*>("widget",0));
        if ( button && val.is<std::string>() )
        {
            button->setText(val.get<std::string>());
        }
    }
    
    bool Button::onClick( const CEGUI::EventArgs& evt )
    {
        VarMap args;
        args.setVar<std::string>("button",getEntity()->getName());
        getEntity()->getApp()->publish("buttonClicked",args);
        
        return true;
    }
    
}
