#ifndef ASPECT_H
#define ASPECT_H

#include "FuncMap.h"

#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>

namespace fhe
{
    class Aspect;
    
    typedef Poco::AutoPtr<Aspect> AspectPtr;
    typedef std::map<std::string,AspectPtr> AspectMap;
    
    class Aspect : public FuncMap, public Poco::RefCountedObject
    {
        private:
            std::string m_name;
            
            static bool m_pythonInitialized;
            
            static boost::python::object m_mainModule, m_mainNamespace;
            
            static void initializePython();
            
        public:
            Aspect( const std::string& name );
            
            boost::python::object getAttr( const std::string& name );
            
            void setAttr( const std::string& name, boost::python::object obj );
            
            void runScript( const std::string& filename );
            
            boost::python::object toPy();
            
            std::string getName();
            
            FuncMap::FuncClosure func( boost::python::object tret, boost::python::object targ );
            
            static boost::python::object defineClass();
    };
    
}

#endif
