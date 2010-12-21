#include <fhe/sim/SpatialNode.h>

namespace fhe
{
    namespace sim
    {
        FHE_NODE( SpatialNode2d );
        FHE_FUNC( SpatialNode2d, getPosition );
        FHE_FUNC( SpatialNode2d, setPosition );
        FHE_FUNC( SpatialNode2d, getRotation );
        FHE_FUNC( SpatialNode2d, setRotation );
        FHE_FUNC( SpatialNode2d, getLocalTransform );
        FHE_FUNC( SpatialNode2d, getGlobalTransform );

        FHE_NODE( SpatialNode2i );
        FHE_FUNC( SpatialNode2i, getPosition );
        FHE_FUNC( SpatialNode2i, setPosition );
        FHE_FUNC( SpatialNode2i, getRotation );
        FHE_FUNC( SpatialNode2i, setRotation );
        FHE_FUNC( SpatialNode2i, getLocalTransform );
        FHE_FUNC( SpatialNode2i, getGlobalTransform );

        FHE_NODE( SpatialNode3d );
        FHE_FUNC( SpatialNode3d, getPosition );
        FHE_FUNC( SpatialNode3d, setPosition );
        FHE_FUNC( SpatialNode3d, getRotation );
        FHE_FUNC( SpatialNode3d, setRotation );
        FHE_FUNC( SpatialNode3d, getLocalTransform );
        FHE_FUNC( SpatialNode3d, getGlobalTransform );
        
        FHE_NODE( SpatialNode3i );
        FHE_FUNC( SpatialNode3i, getPosition );
        FHE_FUNC( SpatialNode3i, setPosition );
        FHE_FUNC( SpatialNode3i, getRotation );
        FHE_FUNC( SpatialNode3i, setRotation );
        FHE_FUNC( SpatialNode3i, getLocalTransform );
        FHE_FUNC( SpatialNode3i, getGlobalTransform );
    }
}
