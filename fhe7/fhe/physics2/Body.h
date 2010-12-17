#ifndef FHE_PHYSICS2_BODY_H
#define FHE_PHYSICS2_BODY_H

#include <fhe/sim/SpatialNode.h>
#include <fhe/core/IOnAttach.h>
#include <Box2D/Box2D.h>

namespace fhe
{
    namespace physics2
    {
        
        class Body : public sim::SpatialNode2, public IOnAttach
        {
            private:
                b2World* m_world;
                b2Body* m_body;
                b2BodyDef m_def;
                
            public:
                Body();
                virtual ~Body();
                
                b2Body* getBody();
                
                void onAttach();
                void onDetach();
                
                Vec2 getPosition();
                void setPosition( Vec2 pos );
                Rot2 getRotation();
                void setRotation( Rot2 rot );
                
        };
        
    }
}

#endif
