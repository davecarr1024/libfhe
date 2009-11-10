#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"
#include <tinyxml.h>

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
            
            EntityMap m_children;
            
            void loadVars( TiXmlHandle h );
            void loadAspects( TiXmlHandle h );
            void loadChildren( TiXmlHandle h );
            void loadTag( TiXmlHandle h, const std::string& tag );
            void loadData( TiXmlHandle h );
            
        public:
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
            
            bool hasFunc( const std::string& name ) const;
            Var call( const std::string& name, const Var& arg = Var() );
            void callAll( const std::string& name, const Var& arg = Var() );
            void publish( const std::string& name, const Var& arg = Var() );
            
            Var onGetVar( const std::string& name ) const;
            void onSetVar( const std::string& name, const Var& val );
            bool onHasVar( const std::string& name ) const;
            
            EntityPtr loadChild( const std::string& filename );
            
            template <class T>
            T getAncestorVar( const std::string& name, const T& def )
            {
                for ( EntityPtr ent = getParent(); ent; ent = ent->getParent() )
                {
                    if ( ent->hasVar<T>(name) )
                    {
                        return ent->getVar<T>(name);
                    }
                }
                return def;
            }
    };
    
}

#endif
