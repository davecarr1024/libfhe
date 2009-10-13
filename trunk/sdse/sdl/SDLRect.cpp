#include "SDLRect.h"

#include "SDL/SDL_opengl.h"

using namespace sdse;

SDLRect::SDLRect(std::string _type, std::string _name) : SDLSceneNode(_type,_name) {}

void SDLRect::geom() {
    glBegin(GL_QUADS);
    glTexCoord2f(0,0);
    glVertex2f(0,0);
    glTexCoord2f(0,1);
    glVertex2f(0,1);
    glTexCoord2f(1,1);
    glVertex2f(1,1);
    glTexCoord2f(1,0);
    glVertex2f(1,0);
    glEnd();
}
