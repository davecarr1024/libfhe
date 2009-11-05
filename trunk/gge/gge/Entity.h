#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"
#include "AutoPtr.h"

#include "tinyxml.h"

#include <stdexcept>

namespace gge
{
    class Entity;
    typedef AutoPtr<Entity> EntityPtr;
    typedef std::map<std::string,EntityPtr> EntityMap;
    typedef std::vector<EntityPtr> EntityList;
    
    class Entity : public VarMap, public RefCounted
    {
        private:
            AspectMap m_aspects;
            
            std::string m_name;
            
            void loadVars( TiXmlHandle h );
            void loadAspects( TiXmlHandle h );
            void loadChildren( TiXmlHandle h );
            
            void loadTag( TiXmlHandle h, const std::string& tag );
            
            Entity* m_parent;
            EntityMap m_children;
            
        public:
            Entity( const std::string& name );
            
            void attachToParent( EntityPtr parent );
            void detachFromParent();
            void addChild( EntityPtr child );
            void removeChild( EntityPtr child );
            
            bool hasChild( const std::string& name );
            EntityPtr getChild( const std::string& name );
            EntityPtr buildChild( const std::string& name );
            EntityPtr loadChild( const std::string& filename, const std::string& name = "Ent" );
            EntityPtr getParent();
            EntityPtr getRoot();
            
            std::string getName();
            
            void addAspect( AspectPtr aspect );
            void removeAspect( AspectPtr aspect );
            bool hasAspect( const std::string& name );
            AspectPtr getAspect( const std::string& name );
            
            AspectPtr buildAspect( const std::string& name );
            
            Var onGetVar( const std::string& name ) const;
            void onSetVar( const std::string& name, const Var& val );
            bool onHasVar( const std::string& name ) const;
            
            void loadData( TiXmlHandle h );
            
            bool hasFunc( const std::string& name ) const;
            
            AbstractFunc* getFunc( const std::string& name ) const;
            
            Var call( const std::string& name, const Var& arg = Var() );
            
            void callAll( const std::string& name, const Var& arg = Var() );
            
            void publish( const std::string& name, const Var& arg = Var() );
    };
    
}

#endif
