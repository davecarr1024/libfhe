#ifndef SCENENODE_H
#define SCENENODE_H

#include <gge/Aspect.h>

#include "Ogre.h"

namespace gge
{
    namespace Graphics
    {
        
        class SceneNode : public Aspect
        {
            public:
                SceneNode();
                ~SceneNode();
                
                Var on_attach( const Var& arg );
                Var on_detach( const Var& arg );
                
                Var set_sceneNodeParent(  const Var& arg  );
                
                Var set_pos(  const Var& arg  );
                Var get_pos( const Var& arg );
                
                Var set_rot( const Var& arg  );
                Var get_rot( const Var& arg );
                
                Var set_scale( const Var& arg  );
                Var get_scale( const Var& arg );
        };

    }
}

#endif
