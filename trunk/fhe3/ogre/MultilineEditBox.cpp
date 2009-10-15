#include "MultilineEditBox.h"

namespace fhe
{
    FHE_ASPECT(MultilineEditBox);
    
    MultilineEditBox::MultilineEditBox() :
        m_multilineEditBox(0)
    {
        addFunc("set_text",&MultilineEditBox::set_text,this);
        addFunc("get_text",&MultilineEditBox::get_text,this);
        addFunc("set_readOnly",&MultilineEditBox::set_readOnly,this);
        addFunc("get_readOnly",&MultilineEditBox::get_readOnly,this);
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
    
    void MultilineEditBox::scrollToEnd()
    {
        if ( m_multilineEditBox )
        {
            m_multilineEditBox->getVertScrollbar()->setScrollPosition(m_multilineEditBox->getVertScrollbar()->getDocumentSize());
        }
    }
    
    void MultilineEditBox::set_readOnly( Var val )
    {
        if ( m_multilineEditBox && val.is<bool>() )
        {
            m_multilineEditBox->setReadOnly(val.get<bool>());
        }
    }
    
    Var MultilineEditBox::get_readOnly()
    {
        Var val;
        if ( m_multilineEditBox )
        {
            val.set<bool>(m_multilineEditBox->isReadOnly());
        }
        return val;
    }
}
