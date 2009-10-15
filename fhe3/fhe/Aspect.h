#ifndef ASPECT_H
#define ASPECT_H

#include "FuncMap.h"

#include "VarMap.h"

#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>

#include "tinyxml.h"

namespace fhe
{
    class Aspect;
    
    typedef Poco::AutoPtr<Aspect> AspectPtr;
    typedef std::map<std::string,AspectPtr> AspectMap;
    
    class Entity;
    typedef Poco::AutoPtr<Entity> EntityPtr;
    
    class Aspect : public FuncMap, public Poco::RefCountedObject
    {
        private:
            std::string m_name;
            
            static bool m_pythonInitialized;
            
            static boost::python::object m_mainModule, m_mainNamespace;
            
            static void initializePython();
            
            Entity* m_entity;
            
        public:
            Aspect();
            
            void init( const std::string& name );
            
            boost::python::object getAttr( const std::string& name );
            
            boost::python::dict defaultNamespace();
            
            static boost::python::dict emptyNamespace();
            
            void runScript( const std::string& filename, boost::python::dict ns );
            
            void runScript( const std::string& filename );
            
            static boost::python::object tryEvalScript( const std::string& s, boost::python::dict ns );
            
            static void execScript( const std::string& s, boost::python::dict ns );
            
            boost::python::object toPy();
            
            std::string getName();
            
            std::string getPath();
            
            FuncMap::FuncClosure func( boost::python::object tret, boost::python::object targ );
            
            void setEntity(EntityPtr entity);
            
            EntityPtr getEntity();
            
            EntityPtr getParentEntity();
            
            static boost::python::object defineClass();
            
            void load( TiXmlHandle h );

            void log( const char* fmt, ...);
            
            void error( const char* fmt, ...);
            
            void pyLog( const std::string& s );
    };
}

#include "AspectFactory.h"
#include "Entity.h"

#endif
