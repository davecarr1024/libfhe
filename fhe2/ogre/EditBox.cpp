#include "EditBox.h"

namespace fhe
{
    FHE_NODE_IMPL(EditBox);
    
    EditBox::EditBox() :
        m_editBox(0)
    {
        addFunc("set_text",&EditBox::set_text,this);
        addFunc("get_text",&EditBox::get_text,this);
    }
    
    CEGUI::Window* EditBox::create( CEGUI::WindowManager* windowManager )
    {
        m_editBox = (CEGUI::Editbox*)windowManager->createWindow("TaharezLook/Editbox",getPath());
        m_editBox->subscribeEvent(CEGUI::Editbox::EventTextAccepted,
            CEGUI::Event::Subscriber(&EditBox::onAccepted,this));
        return m_editBox;
    }
    
    void EditBox::set_text( Var val )
    {
        if ( m_editBox && val.is<std::string>() )
        {
            m_editBox->setText( val.get<std::string>() );
        }
    }
    
    Var EditBox::get_text()
    {
        Var var;
        if ( m_editBox )
        {
            var.set<std::string>(m_editBox->getText().c_str());
        }
        return var;
    }
    
    bool EditBox::onAccepted( const CEGUI::EventArgs& evt )
    {
        VarMap args;
        args.setVar("path",getPath());
        args.setVar("text",getVar<std::string>("text",""));
        getRoot()->publish("editBoxAccepted",args);
    }
}
