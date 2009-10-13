#include "SDLGraphicsServer.h"
#include "SDLSceneNode.h"
#include "core/Entity.h"
#include "core/App.h"

using namespace sdse;

SDLGraphicsServer::SDLGraphicsServer(App* _app) :
    AppListener(_app),
    screen(0),
    lastFrameTime(0),
    frameTime(0),
    rootSceneNode(0),
    numFrames(0) {

    int screenw = 640,
        screenh = 480;
    bool fullscreen = false;
    float fps = 60;
    frameTime = 1.0f / fps;
        
    if (SDL_Init(SDL_INIT_VIDEO)) {
        printf("ERROR: couldn't init sdl: %s\n",SDL_GetError());
        return;
    }
    
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,1);
    
    Uint32 flags = SDL_OPENGL | SDL_HWSURFACE | SDL_DOUBLEBUF;
    if (fullscreen) flags |= SDL_FULLSCREEN;
    
    screen = SDL_SetVideoMode(screenw,screenh,32,flags);
    if (!screen) {
        printf("ERROR: couldn't initialize screen: %s\n",SDL_GetError());
        return;
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
    
    Entity* entity = new Entity("RootSceneNode");
    app->addEntity(entity);
    rootSceneNode = new SDLSceneNode("sdl/SDLSceneNode","RootSceneNode");
    entity->addAspect(rootSceneNode);
}

SDLSceneNode* SDLGraphicsServer::getRootSceneNode() {
    return rootSceneNode;
}

void SDLGraphicsServer::update(float time, float dtime) {
    if (!numFrames++) lastFrameTime = time;
    
    SDL_Event event;
    
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                app->shutdown();
                break;
        }
    }
    
    if (time - lastFrameTime > frameTime) {
        lastFrameTime += frameTime;
        
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        rootSceneNode->render();
        
        glFlush();
        SDL_GL_SwapBuffers();
    }
}
