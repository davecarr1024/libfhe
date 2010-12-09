#include <test_mod/TestNode.h>

namespace test_mod
{
    FHE_MOD( test_mod );
    
    FHE_NODE( TestNode );
    FHE_FUNC( TestNode, setMsg );
    FHE_FUNC( TestNode, getMsg );
    FHE_VAR( TestNode, m_i );
    
    FHE_END_MOD();
}
