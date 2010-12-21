#ifndef FHE_PHYSICS2_BOX_H
#define FHE_PHYSICS2_BOX_H

#include <fhe/physics2/Shape.h>

namespace fhe
{
    namespace physics2
    {
        
        class Box : public Shape
        {
            private:
                Vec2d m_size;
                Vec2d m_pos;
                Rot2d m_rot;
                
            protected:
                b2Shape* build();
                
            public:
                Box();
                virtual ~Box();
                
                void setSize( Vec2d size );
                Vec2d getSize();
                void setPosition( Vec2d pos );
                Vec2d getPosition();
                void setRotation( Rot2d rot );
                Rot2d getRotation();
        };
        
    }
}

#endif
