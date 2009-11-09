#ifndef MATERIAL_MANAGER_H
#define MATERIAL_MANAGER_H

#include <SDL/SDL_opengl.h>
#include <map>
#include <string>

#include <fhe/VarMap.h>

namespace fhe
{
    namespace Graphics
    {
        
        class MaterialManager 
        {
            private:
                MaterialManager();
                
                MaterialManager( const MaterialManager& mm ) {}
                void operator=( const MaterialManager& mm ) {}
                
                std::map<std::string,GLuint> m_textures;
                
            public:
                static MaterialManager& instance();
                
                GLuint loadTexture( const std::string& filename );
                
                void bindTexture( const std::string& filename );
                
                void bindTexture( GLuint texture );
                
                void unbindTexture();
                
                void bind( const VarMap& args );
                
                void unbind();
        };
        
    }
}

#endif
