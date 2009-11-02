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
            
            static bool m_pythonInitialized;
            static boost::python::object m_mainModule, m_mainNamespace;
            
            static void initializePython();
            
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
            
            boost::python::object getAttr( const std::string& name );
            void setAttr( const std::string& name, boost::python::object val );
            
            static boost::python::object defineClass();
            
            boost::python::object toPy();
            
            static boost::python::dict defaultNamespace();
            boost::python::dict selfNamespace();
            
            void runScript( const std::string& name );
            
            void execScript( const std::string& s, boost::python::dict ns );
            
            boost::python::object evalScript( const std::string& s, boost::python::dict ns );
    };
    
}

#include "AspectFactory.h"

#endif
