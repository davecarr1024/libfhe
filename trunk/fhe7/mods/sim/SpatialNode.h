#ifndef FHE_SIM_SPATIAL_NODE_H
#define FHE_SIM_SPATIAL_NODE_H

#include <fhe/Node.h>
#include <fhe/Mat.h>
#include <fhe/Vec.h>
#include <fhe/Rot.h>

namespace fhe
{
    
    template <size_t dim>
    class SpatialNode : public Node
    {
        public:
            typedef Mat<dim> M;
            typedef Vec<dim> V;
            typedef Rot<dim> R;
            
            V position;
            R rotation;
            
            M localTransform()
            {
                return M::translation( position ) * M::rotation( rotation );
            }
            
            M globalTransform()
            {
                M parentTransform;
                if ( ancestorCall( &SpatialNode<dim>::globalTransform, parentTransform ) )
                {
                    return parentTransform * localTransform();
                }
                else
                {
                    return localTransform();
                }
            }
    };
    
    typedef SpatialNode<2> SpatialNode2;
    typedef SpatialNode<3> SpatialNode3;
    
}

#endif
