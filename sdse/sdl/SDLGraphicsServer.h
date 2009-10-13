#ifndef SDLGRAPHICSSERVER_H
#define SDLGRAPHICSSERVER_H

///sdse_lib SDL
///sdse_lib SDL_image
///sdse_lib GL
///sdse_lib GLU

#include "SDL/SDL.h"
#include "SDL/SDL_opengl.h"
#include "SDL/SDL_image.h"

#include "core/AppListener.h"

namespace sdse {
    
    class SDLSceneNode;
    
    class SDLGraphicsServer : public AppListener {
        private:
            SDL_Surface* screen;
            SDLSceneNode* rootSceneNode;

            float frameTime, lastFrameTime;
            int numFrames;
            
        public: 
            SDLGraphicsServer(App* _app);
            
            void update(float time, float dtime);
            
            SDLSceneNode* getRootSceneNode();
    };
    
}

#endif
