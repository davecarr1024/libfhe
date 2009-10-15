#include "Console.h"
#include <fhe/math/Vec2.h>

namespace fhe
{
    FHE_ASPECT(Console);
    
    Console::Console()
    {
        addFunc("on_attach",&Console::on_attach,this);
        addFunc("msg_editBoxAccepted",&Console::msg_editBoxAccepted,this);
    }

    void Console::on_attach()
    {
        FrameWindow::on_attach();
        
        m_input = getEntity()->buildChild("input")->addAspect("EditBox").cast<EditBox>();
        m_input->getEntity()->setVar("pos",Vec2(0.05,0.75));
        m_input->getEntity()->setVar("size",Vec2(0.9,0.2));
        
        m_output = getEntity()->buildChild("output")->addAspect("MultilineEditBox").cast<MultilineEditBox>();
        m_output->getEntity()->setVar("pos",Vec2(0.05,0.1));
        m_output->getEntity()->setVar("size",Vec2(0.9,0.65));
        m_output->getEntity()->setVar("readOnly",true);
    }
    
    void Console::appendOutput( const std::string& text )
    {
        if ( m_output )
        {
            m_output->getEntity()->setVar<std::string>("text",m_output->getEntity()->getVar<std::string>("text","") + text);
            m_output->scrollToEnd();
        }
    }
    
    void Console::msg_editBoxAccepted( VarMap args )
    {
        std::string path = args.getVar<std::string>("path",""), text = args.getVar<std::string>("text","");
        if ( m_input && m_input->getPath() == path )
        {
            m_input->getEntity()->setVar<std::string>("text","");
//             appendOutput(text);
            on_text(text);
        }
    }
}
