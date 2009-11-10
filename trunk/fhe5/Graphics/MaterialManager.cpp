#include "MaterialManager.h"
#include <fhe/FileSystem.h>
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>
#include <fhe/math/Rot.h>

#include <SDL/SDL.h>
#include <SDL/SDL_image.h>

#include <FTGL/FTGLPixmapFont.h>

#include <stdexcept>

namespace fhe
{
    namespace Graphics
    {
        
        MaterialManager::MaterialManager()
        {
        }
        
        MaterialManager::~MaterialManager()
        {
            for ( std::map<std::string,FTFont*>::iterator i = m_fonts.begin(); i != m_fonts.end(); ++i )
            {
                delete i->second;
            }
            m_fonts.clear();
        }
        
        MaterialManager& MaterialManager::instance()
        {
            static MaterialManager mm;
            return mm;
        }
        
        GLuint MaterialManager::loadTexture( const std::string& filename )
        {
            if ( m_textures.find(filename) == m_textures.end() )
            {
                GLuint texture;
                GLint numColors;
                GLenum textureFormat;
                
                SDL_Surface* image = IMG_Load(FileSystem::instance().getFile(filename).c_str());
                if ( !image )
                {
                    throw std::runtime_error("unable to load texture " + filename + ": " + SDL_GetError() );
                }
                
                numColors = image->format->BytesPerPixel;                                            
                switch (numColors) {                                                                 
                    case 3:                                                                          
                        textureFormat = GL_RGB;                                                      
                        break;                                                                       
                    case 4:                                                                          
                        textureFormat = GL_RGBA;                                                     
                        break;                                                                       
                    default:                                                                         
                        throw std::runtime_error("invalid image format in file " + filename );
                }                                                                                    
                                                                                                    
                glGenTextures(1,&texture);
                glBindTexture(GL_TEXTURE_2D,texture);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
                gluBuild2DMipmaps(GL_TEXTURE_2D,numColors,image->w,image->h,textureFormat,GL_UNSIGNED_BYTE,image->pixels);
                glBindTexture(GL_TEXTURE_2D,0);

                m_textures[filename] = texture;
            }
            return m_textures[filename];
        }
        
        void MaterialManager::bindTexture( GLuint texture )
        {
            glBindTexture(GL_TEXTURE_2D,texture);
        }
        
        void MaterialManager::bindTexture( const std::string& filename )
        {
            if ( filename.empty() )
            {
                unbindTexture();
            }
            else
            {
                bindTexture(loadTexture(filename));
            }
        }
        
        void MaterialManager::unbindTexture()
        {
            glBindTexture(GL_TEXTURE_2D,0);
        }
        
        void MaterialManager::bind( const VarMap& args )
        {
            glPushAttrib(GL_TEXTURE_BIT | GL_CURRENT_BIT);
            glMatrixMode(GL_TEXTURE);
            glPushMatrix();
            
            if ( args.hasVar<std::string>("texture") )
            {
                bindTexture(args.getVar<std::string>("texture"));
            }
            if ( args.hasVar<Color>("color") )
            {
                Color c = args.getVar<Color>("color");
                glColor4f(c.r,c.g,c.b,c.a);
            }
            if ( args.hasVar<Vec2>("pos") )
            {
                Vec2 pos = args.getVar<Vec2>("pos");
                glTranslatef(pos.x,pos.y,0);
            }
            if ( args.hasVar<Rot>("rot") )
            {
                Rot rot = args.getVar<Rot>("rot");
                glRotatef(rot.degrees(),0,0,-1);
            }
            if ( args.hasVar<Vec2>("scale") )
            {
                Vec2 scale = args.getVar<Vec2>("scale");
                glScalef(scale.x,scale.y,1);
            }
            
            glMatrixMode(GL_MODELVIEW);
        }
        
        void MaterialManager::unbind()
        {
            glPopAttrib();
            glMatrixMode(GL_TEXTURE);
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
        }
        
        FTFont* MaterialManager::loadFont( const std::string& filename )
        {
            if ( m_fonts.find(filename) == m_fonts.end() )
            {
                FTFont* font = new FTPixmapFont(FileSystem::instance().getFile(filename).c_str());
                if ( !font )
                {
                    throw std::runtime_error("unable to load font " + filename);
                }
                m_fonts[filename] = font;
            }
            return m_fonts[filename];
        }
    }
}
