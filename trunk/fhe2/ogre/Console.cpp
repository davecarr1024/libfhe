#include "Console.h"
#include <fhe/math/Vec2.h>

namespace fhe
{
    FHE_NODE_IMPL(Console);
    
    Console::Console()
    {
        addFunc("on_attach",&Console::on_attach,this);
        addFunc("msg_editBoxAccepted",&Console::msg_editBoxAccepted,this);
    }

    void Console::on_attach()
    {
        FrameWindow::on_attach();
        
        m_input = boost::dynamic_pointer_cast<EditBox,Node>(NodeFactory::instance().buildNode("EditBox","input"));
        addChild(m_input);
        m_input->setVar("pos",Vec2(0.05,0.75));
        m_input->setVar("size",Vec2(0.9,0.2));
        
        m_output = boost::dynamic_pointer_cast<MultilineEditBox,Node>(NodeFactory::instance().buildNode("MultilineEditBox","output"));
        addChild(m_output);
        m_output->setVar("pos",Vec2(0.05,0.1));
        m_output->setVar("size",Vec2(0.9,0.65));
        m_output->setVar("readOnly",true);
    }
    
    void Console::appendOutput( const std::string& text )
    {
        if ( m_output )
        {
            m_output->setVar<std::string>("text",m_output->getVar<std::string>("text","") + text);
            m_output->scrollToEnd();
        }
    }
    
    void Console::msg_editBoxAccepted( VarMap args )
    {
        std::string path = args.getVar<std::string>("path",""), text = args.getVar<std::string>("text","");
        if ( m_input && m_input->getPath() == path )
        {
            m_input->setVar<std::string>("text","");
//             appendOutput(text);
            on_text(text);
        }
    }
}
