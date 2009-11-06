#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"

namespace fhe
{
    
    class Entity;
    
    typedef AutoPtr<Entity> EntityPtr;
    typedef std::vector<EntityPtr> EntityList;
    typedef std::map<std::string,EntityPtr> EntityMap;
    
    class Entity : public VarMap
    {
        private:
            static std::map<std::string,int> m_nameCounts;
            static std::string makeName( const std::string& name );
        
            int m_refCount;
            
            std::string m_name;
            
            Entity* m_parent;
            
            AspectMap m_aspects;
            
        public:
            EntityMap m_children;
            
            Entity( const std::string& name );
            
            void incRef();
            bool decRef();
            
            std::string getName();
            std::string getPath();
            
            void attachToParent( EntityPtr parent );
            void detachFromParent();
            EntityPtr getParent();
            EntityPtr getRoot();
            
            void addChild( EntityPtr child );
            void removeChild( EntityPtr child );
            bool hasChild( const std::string& name );
            EntityPtr getChild( const std::string& name );
            EntityPtr buildChild( const std::string& name );
            
            void addAspect( AspectPtr aspect );
            void removeAspect( AspectPtr aspect );
            bool hasAspect( const std::string& name );
            AspectPtr getAspect( const std::string& name );
            AspectPtr buildAspect( const std::string& name );
            
            bool hasFunc( const std::string& name );
            Var call( const std::string& name, const Var& arg = Var() );
            void callAll( const std::string& name, const Var& arg = Var() );
            void publish( const std::string& name, const Var& arg = Var() );
    };
    
}

#endif
