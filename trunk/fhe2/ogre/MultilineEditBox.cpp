#include "MultilineEditBox.h"

namespace fhe
{
    FHE_NODE_IMPL(MultilineEditBox);
    
    MultilineEditBox::MultilineEditBox() :
        m_multilineEditBox(0)
    {
        addFunc("set_text",&MultilineEditBox::set_text,this);
        addFunc("get_text",&MultilineEditBox::get_text,this);
    }
    
    CEGUI::Window* MultilineEditBox::create( CEGUI::WindowManager* windowManager )
    {
        m_multilineEditBox = (CEGUI::MultiLineEditbox*)windowManager->createWindow("TaharezLook/MultiLineEditbox",getPath());
        return m_multilineEditBox;
    }
    
    void MultilineEditBox::set_text( Var val )
    {
        if ( m_multilineEditBox && val.is<std::string>() )
        {
            m_multilineEditBox->setText( val.get<std::string>() );
        }
    }
    
    Var MultilineEditBox::get_text()
    {
        Var var;
        if ( m_multilineEditBox )
        {
            var.set<std::string>(m_multilineEditBox->getText().c_str());
        }
        return var;
    }
}
