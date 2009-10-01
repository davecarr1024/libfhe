#ifndef CONSOLE_H
#define CONSOLE_H

#include "FrameWindow.h"
#include "EditBox.h"
#include "MultilineEditBox.h"

namespace fhe
{
    
    class Console : public FrameWindow
    {
        private:
            EditBoxPtr m_input;
            MultilineEditBoxPtr m_output;
            
        public:
            Console();
            
            void on_attach();
            
            void msg_editBoxAccepted( VarMap args );
            
            virtual void on_text( const std::string& text ) {}
            
            void appendOutput( const std::string& text );
    };
    
    FHE_NODE_DECL(Console);
    
}

#endif
