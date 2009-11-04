#ifndef ASPECT_H
#define ASPECT_H

#include "FuncMap.h"
#include "AutoPtr.h"

#include "tinyxml.h"

#include <map>
#include <string>
#include <vector>

namespace gge
{
    class Aspect;
    typedef AutoPtr<Aspect> AspectPtr;
    typedef std::map<std::string,AspectPtr> AspectMap;
    typedef std::vector<AspectPtr> AspectList;
    
    class Entity;
    
    class Aspect : public FuncMap, public RefCounted
    {
        private:
            std::string m_name;
            
            Entity* m_entity;
            
        public:
            Aspect();
            ~Aspect();
            
            void init( const std::string& name );
            std::string getName();
            std::string getPath();
            
            void log( const char* format, ... );
            void error( const char* format, ... );
            
            Entity* getEntity();
            
            void attachToEntity( Entity* entity );
            void detachFromEntity();
            
            void loadData( TiXmlHandle h );
    };
    
}

#include "AspectFactory.h"
#include "Entity.h"

#endif
