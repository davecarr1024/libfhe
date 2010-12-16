#include <fhe/sim/SpatialNode.h>

namespace fhe
{
    namespace sim
    {
        FHE_NODE( SpatialNode2 );
        FHE_FUNC( SpatialNode2, getPosition );
        FHE_FUNC( SpatialNode2, setPosition );
        FHE_FUNC( SpatialNode2, getRotation );
        FHE_FUNC( SpatialNode2, setRotation );
        FHE_FUNC( SpatialNode2, getLocalTransform );
        FHE_FUNC( SpatialNode2, getGlobalTransform );

        FHE_NODE( SpatialNode3 );
        FHE_FUNC( SpatialNode3, getPosition );
        FHE_FUNC( SpatialNode3, setPosition );
        FHE_FUNC( SpatialNode3, getRotation );
        FHE_FUNC( SpatialNode3, setRotation );
        FHE_FUNC( SpatialNode3, getLocalTransform );
        FHE_FUNC( SpatialNode3, getGlobalTransform );
    }
}
