#ifndef FHE_SIM_SPATIAL_NODE_H
#define FHE_SIM_SPATIAL_NODE_H

#include <fhe/Node.h>
#include <fhe/Mat.h>
#include <fhe/Vec.h>
#include <fhe/Rot.h>

namespace fhe
{
    namespace sim
    {
        
        template <size_t dim>
        class SpatialNode : public Node
        {
            public:
                typedef Mat<dim> M;
                typedef Vec<dim> V;
                typedef Rot<dim> R;
                
            private:
                V m_pos;
                R m_rot;
                
            public:
                virtual void setPosition( V pos )
                {
                    m_pos = pos;
                }
                
                virtual V getPosition()
                {
                    return m_pos;
                }
                
                virtual void setRotation( R rot )
                {
                    m_rot = rot;
                }
                
                virtual R getRotation()
                {
                    return m_rot;
                }
                
                virtual M getLocalTransform()
                {
                    return M::translation( m_pos ) * M::rotation( m_rot );
                }
                
                virtual M getGlobalTransform()
                {
                    M parentTransform;
                    if ( ancestorCall( &SpatialNode<dim>::getGlobalTransform, parentTransform ) )
                    {
                        return parentTransform * getLocalTransform();
                    }
                    else
                    {
                        return getLocalTransform();
                    }
                }
        };
        
        typedef SpatialNode<2> SpatialNode2;
        typedef SpatialNode<3> SpatialNode3;

    }
}

#endif
