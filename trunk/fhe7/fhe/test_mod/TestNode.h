#ifndef TEST_MOD_NODE_H
#define TEST_MOD_NODE_H

#include <fhe/core/Node.h>

namespace test_mod
{
    
    class TestVar
    {
        public:
            TestVar( int _i = 0 ) :
                i( _i )
            {
            }
            
            int i;
    };
    
    void operator>>( const YAML::Node& node, TestVar& v );
    YAML::Emitter& operator<<( YAML::Emitter& out, const TestVar& v );
    
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
            TestVar var;
    };
    
}

namespace fhe
{
    FHE_CAN_SERIALIZE( test_mod::TestVar );
}

#endif
