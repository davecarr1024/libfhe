#ifndef FHE_PHYSICS2_SHAPE_H
#define FHE_PHYSICS2_SHAPE_H

#include <fhe/sim/SpatialNode.h>
#include <fhe/core/IOnAttach.h>
#include <Box2D/Box2D.h>

namespace fhe
{
    namespace physics2
    {
        
        class Shape : public sim::SpatialNode2d, public IOnAttach
        {
            private:
                b2Body* m_body;
                b2Fixture* m_fixture;
                double m_density;
                
            protected:
                virtual b2Shape* build() = 0;
                
                bool built() const;
                
            public:
                Shape();
                virtual ~Shape();
                
                void onAttach();
                void onDetach();
                
                void setDensity( double density );
                double getDensity();
        };
        
    }
}

#endif
