#include <fhe/test_mod/TestNode.h>

namespace test_mod
{
    void operator>>( const YAML::Node& node, TestVar& v )
    {
        node >> v.i;
    }
    
    YAML::Emitter& operator<<( YAML::Emitter& out, const TestVar& v )
    {
        out << v.i;
        return out;
    }
    
    FHE_NODE( TestNode );
    FHE_FUNC( TestNode, setMsg );
    FHE_FUNC( TestNode, getMsg );
    FHE_VAR( TestNode, m_i );
    FHE_VAR( TestNode, var );
}
