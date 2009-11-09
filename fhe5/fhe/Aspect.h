#ifndef ASPECT_H
#define ASPECT_H

#include "Func.h"
#include "AutoPtr.h"

#include <tinyxml.h>
#include <string>
#include <vector>
#include <map>

namespace fhe
{
    class Entity;
    typedef AutoPtr<Entity> EntityPtr;
    
    class Aspect;
    
    typedef AutoPtr<Aspect> AspectPtr;
    typedef std::vector<AspectPtr> AspectList;
    typedef std::map<std::string,AspectPtr> AspectMap;
    
    class Aspect
    {
        private:
            int m_refCount;
            
            std::map<std::string,std::vector<AbstractFunc*> > m_funcs;
            
            Entity* m_entity;
            
            std::string m_name;
            
        public:
            Aspect();
            virtual ~Aspect();
            
            void init( const std::string& name );
            
            std::string getName();
            std::string getPath();
            
            void incRef();
            bool decRef();
            
            bool hasFunc( const std::string& name );
            
            AbstractFunc* getFunc( const std::string& name );
            
            void addFunc( AbstractFunc* func );
            
            Var call( const std::string& name, const Var& arg = Var() );
            
            void attachToEntity( EntityPtr entity );
            void detachFromEntity();
            EntityPtr getEntity();
            
            void load( TiXmlHandle h );
            
            void log( const char* fmt, ... );
            void error( const char* fmt, ... );
    };
    
}

#include "AspectFactory.h"
#include "Entity.h"

#endif
