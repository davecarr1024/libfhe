#include "Window.h"
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

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
            
            Vec2 res = getEntity()->getVar<Vec2>("res",Vec2(800,600));
            
            m_screen = SDL_SetVideoMode(res.x,res.y,32,flags);
                
            if ( !m_screen )
            {
                error("error initializing screen: %s",SDL_GetError() );
            }
            
            glEnable(GL_TEXTURE_2D);
            glShadeModel(GL_SMOOTH);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
            glLineWidth(getEntity()->getVar<int>("lineWidth",1));
            Color clearColor = getEntity()->getVar<Color>("clearColor",Color(1,1,1,1));
            glClearColor(clearColor.r,clearColor.g,clearColor.b,clearColor.a);
            
            return Var();
        }
        
        void Window::set2dProjection()
        {
            Vec2 res = getEntity()->getVar<Vec2>("res",Vec2(800,600));
                
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluOrtho2D(0,res.x,0,res.y);
            
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glTranslatef(0,res.y,0);
            glScalef(res.x,-res.y,1);
            
            glColor4f(1,1,1,1);
            glBindTexture(GL_TEXTURE_2D,0);
            
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_NORMALIZE);
        }
        
        void Window::set3dProjection()
        {
            Vec2 res = getEntity()->getVar<Vec2>("res",Vec2(800,600));
                
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45,float(res.x)/float(res.y),1,1000);
            
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            
            glColor4f(1,1,1,1);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_NORMALIZE);
        }
        
        FHE_FUNC_IMPL(Window,msg_update)
        {
            float time = arg.get<float>(0);
            static float lastTime = time;
            int fps = getEntity()->getVar<int>("fps",60);
            float dtime = fps > 0 ? 1.0 / float(fps) : 0;
            Vec2 res = getEntity()->getVar<Vec2>("res",Vec2(800,600));
            
            SDL_Event event;
            while ( SDL_PollEvent(&event) )
            {
                switch ( event.type )
                {
                    case SDL_QUIT:
                        getEntity()->getRoot()->callNoRetNoArg("shutdown");
                        break;
                    case SDL_KEYDOWN:
                        if ( event.key.keysym.sym == SDLK_ESCAPE )
                        {
                            getEntity()->getRoot()->callNoRetNoArg("shutdown");
                        }
                        getEntity()->publish<int>("keyDown",event.key.keysym.sym);
                        break;
                    case SDL_KEYUP:
                        getEntity()->publish<int>("keyUp",event.key.keysym.sym);
                        break;
                    case SDL_MOUSEMOTION:
                        getEntity()->publish<Vec2>("mouseMotion",Vec2(event.motion.x/res.x,event.motion.y/res.y));
                        break;
                    case SDL_MOUSEBUTTONDOWN:
                        mouseButton(event.button.button,event.button.x,event.button.y,"Down");
                        break;
                    case SDL_MOUSEBUTTONUP:
                        mouseButton(event.button.button,event.button.x,event.button.y,"Up");
                        break;
                }
            }
                
            if ( time - lastTime > dtime )
            {
                lastTime += dtime;
                
                glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
                
                set3dProjection();
                getEntity()->publish("render3");
                
                set2dProjection();
                getEntity()->publish("render2");
                
                glFlush();
                SDL_GL_SwapBuffers();
            }
            return Var();
        }
        
        void Window::mouseButton( int button, int x, int y, const std::string& dir )
        {
            Vec2 res = getEntity()->getVar<Vec2>("res",Vec2(800,600));
            VarMap args;
            args.setVar<int>("button",button);
            args.setVar<Vec2>("pos",Vec2(float(x)/res.x,float(y)/res.y));
            getEntity()->publish<VarMap>("mouseButton" + dir,args);
        }
    }
}
