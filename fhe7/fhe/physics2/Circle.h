#ifndef FHE_PHYSICS2_CIRCLE_H
#define FHE_PHYSICS2_CIRCLE_H

#include <fhe/physics2/Shape.h>

namespace fhe
{
    namespace physics2
    {
        class Circle : public Shape
        {
            private:
                double m_radius;
                
            protected:
                b2Shape* build();
                
            public:
                Circle();
                ~Circle();
                
                void setRadius( double radius );
                double getRadius();
                void setPosition( Vec2 position );
        };
    }
}

#endif
