#include "Window.h"

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Window,Aspect);
        
        FHE_FUNC_IMPL(Window,on_attach)
        {
            if ( SDL_Init(SDL_INIT_VIDEO) )
            {
                error( "error initializing sdl: %s", SDL_GetError() );
            }
            
            SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
            
            Uint32 flags = SDL_OPENGL | SDL_HWSURFACE | SDL_DOUBLEBUF;
            if ( getEntity()->getVar<bool>("fullscreen",false) )
            {
                flags |= SDL_FULLSCREEN;
            }
            
            int screenw = getEntity()->getVar<int>("screenw",800),
                screenh = getEntity()->getVar<int>("screenh",600);
            
            m_screen = SDL_SetVideoMode(screenw,screenh,32,flags);
                
            if ( !m_screen )
            {
                error("error initializing screen: %s",SDL_GetError() );
            }
            
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
            
            glViewport(0,0,screenw,screenh);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0,screenw,screenh,0,-1,1);
            
            glClearColor(1,1,1,1);
        }
        
        FHE_FUNC_IMPL(Window,msg_update)
        {
            float time = arg.get<float>(0);
        }
        
    }
}
