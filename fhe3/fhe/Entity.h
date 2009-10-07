#ifndef ENTITY_H
#define ENTITY_H

#include "VarMap.h"
#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>

namespace fhe
{
    class Aspect;
    
    typedef Poco::AutoPtr<Aspect> AspectPtr;
    typedef std::map<std::string,AspectPtr> AspectMap;

    class Entity;
    
    typedef Poco::AutoPtr<Entity> EntityPtr;
    typedef std::map<std::string,EntityPtr> EntityMap;
    
    class Entity : public VarMap, public Poco::RefCountedObject
    {
        private:
            Entity( const Entity& entity ) {}
            Entity& operator=( const Entity& entity ) {}
            
            Entity* m_parent;
            EntityMap m_children;
            
            std::string m_name, m_path;
            
            AspectMap m_aspects;
            
        public:
            static EntityPtr root;
            
            Entity( const std::string& name );
            ~Entity();
            
            void attachToParent( EntityPtr parent );
            void detachFromParent();
            void addChild( EntityPtr child );
            void removeChild( EntityPtr child );
            
            bool hasChild( const std::string& name );
            EntityPtr getChild( const std::string& name );
            EntityPtr getParent();
    };
    
}

#endif
