#ifndef SDLRECT_H
#define SDLRECT_H

///sdse_aspect SDLRect

#include "SDLSceneNode.h"

namespace sdse {

    class SDLRect : public SDLSceneNode {
        public:
            SDLRect(std::string _type, std::string _name);
            
            void geom();
    };
    
}

#endif
