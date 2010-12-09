#ifndef TEST_MOD_NODE_H
#define TEST_MOD_NODE_H

#include <fhe/Node.h>

namespace test_mod
{
    
    class TestNode : public fhe::Node
    {
        private:
            std::string m_msg;
            
        public:
            void setMsg( std::string msg )
            {
                m_msg = msg;
            }
            
            std::string getMsg()
            {
                return m_msg;
            }
            
            int m_i;
    };
    
}

#endif
