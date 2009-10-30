#ifndef APP_H
#define APP_H

#include "Entity.h"

namespace gge
{
    
    class App
    {
        private:
            EntityMap m_entities;
            
        public:
            App();
            
            void addEntity( EntityPtr entity );
            void removeEntity( EntityPtr entity );
            bool hasEntity( const std::string& name );
            EntityPtr getEntity( const std::string& name );
            
            EntityPtr buildEntity( const std::string& name );
            
            void publish( const std::string& cmd, const VarMap& args );
    };
    
}

#endif
