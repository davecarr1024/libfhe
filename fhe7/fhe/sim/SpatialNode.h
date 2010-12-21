#ifndef FHE_SIM_SPATIAL_NODE_H
#define FHE_SIM_SPATIAL_NODE_H

#include <fhe/core/Node.h>
#include <fhe/core/Mat.h>
#include <fhe/core/Vec.h>
#include <fhe/core/Rot.h>

namespace fhe
{
    namespace sim
    {
        
        template <size_t dim, typename T>
        class SpatialNode : public Node
        {
            public:
                typedef Mat<dim> M;
                typedef Vec<dim,T> V;
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
                    return M::translation( getPosition() ) * M::rotation( getRotation() );
                }
                
                virtual M getGlobalTransform()
                {
                    M parentTransform;
                    ancestorCall( &SpatialNode<dim,T>::getGlobalTransform, parentTransform );
                    return parentTransform * getLocalTransform();
                }
        };
        
        typedef SpatialNode<2,double> SpatialNode2;
        typedef SpatialNode<3,double> SpatialNode3;

    }
}

#endif
