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
    
    class App;
    
    class Entity : public VarMap, public RefCounted
    {
        private:
            AspectMap m_aspects;
            
            std::string m_name;
            
            App* m_app;
            
            void loadVars( TiXmlHandle h );
            void loadAspects( TiXmlHandle h );
            
            void loadTag( TiXmlHandle h, const std::string& tag );
            
        public:
            Entity( const std::string& name );
            
            std::string getName();
            
            App* getApp();
            
            void addAspect( AspectPtr aspect );
            void removeAspect( AspectPtr aspect );
            bool hasAspect( const std::string& name );
            AspectPtr getAspect( const std::string& name );
            
            AspectPtr buildAspect( const std::string& name );
            
            void attachToApp( App* app );
            void detachFromApp();
            
            Var onGetVar( const std::string& name ) const;
            void onSetVar( const std::string& name, const Var& val );
            bool onHasVar( const std::string& name ) const;
            
            void loadData( TiXmlHandle h );
            
            bool hasFunc( const std::string& name ) const;
            
            AbstractFunc* getFunc( const std::string& name ) const;
            
            Var call( const std::string& name, const Var& arg = Var() );
            
            void callAll( const std::string& name, const Var& arg = Var() );
    };
    
}

#include "App.h"

#endif
