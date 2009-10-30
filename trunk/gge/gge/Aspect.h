#ifndef ASPECT_H
#define ASPECT_H

#include "FuncMap.h"
#include "AutoPtr.h"

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
            
            Entity* getEntity();
            
            void attachToEntity( Entity* entity );
            void detachFromEntity();
    };
    
}

#include "AspectFactory.h"

#endif
