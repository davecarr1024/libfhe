#ifndef APP_H
#define APP_H

#include "Entity.h"
#include "tinyxml.h"

namespace gge
{
    
    class App
    {
        private:
            EntityMap m_entities;
            bool m_shutdown;
            
        public:
            App();
            
            void addEntity( EntityPtr entity );
            void removeEntity( EntityPtr entity );
            bool hasEntity( const std::string& name );
            EntityPtr getEntity( const std::string& name );
            
            EntityPtr buildEntity( const std::string& name );
            
            void publish( const std::string& cmd, const Var& arg );
            
            void load( const std::string& filename );
            void loadData( TiXmlHandle h );
            
            float getTime();
            void shutdown();
            void run( float maxTime = -1 );
    };
    
}

#endif
