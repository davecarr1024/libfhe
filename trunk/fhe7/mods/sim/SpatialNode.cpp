#include <sim/SpatialNode.h>

namespace fhe
{
    FHE_MOD( sim );
    
    FHE_NODE( SpatialNode2 );
    FHE_VAR( SpatialNode2, position );
    FHE_VAR( SpatialNode2, rotation );
    FHE_FUNC( SpatialNode2, localTransform );
    FHE_FUNC( SpatialNode2, globalTransform );
    
    FHE_NODE( SpatialNode3 );
    FHE_VAR( SpatialNode3, position );
    FHE_VAR( SpatialNode3, rotation );
    FHE_FUNC( SpatialNode3, localTransform );
    FHE_FUNC( SpatialNode3, globalTransform );
    
    FHE_END_MOD
    
}
