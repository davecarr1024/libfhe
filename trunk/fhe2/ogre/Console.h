#ifndef CONSOLE_H
#define CONSOLE_H

#include "Widget.h"
#include "EditBox.h"
#include "MultilineEditBox.h"

namespace fhe
{
    
    class Console : public Widget
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
