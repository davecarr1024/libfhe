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
                Vec2 m_size;
                Vec2 m_pos;
                Rot2 m_rot;
                
            protected:
                b2Shape* build();
                
            public:
                Box();
                virtual ~Box();
                
                void setSize( Vec2 size );
                Vec2 getSize();
                void setPosition( Vec2 pos );
                Vec2 getPosition();
                void setRotation( Rot2 rot );
                Rot2 getRotation();
        };
        
    }
}

#endif
