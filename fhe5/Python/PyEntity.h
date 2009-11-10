#ifndef PYENTITY_H
#define PYENTITY_H

#include <boost/python.hpp>
#include <fhe/Entity.h>
#include "Script.h"

namespace fhe
{
    namespace Python
    {
        
        class PyEntity
        {
            private:
                Script* m_script;
                Entity* m_entity;
                
                class PyCall
                {
                    private:
                        AbstractFunc* m_func;
                        
                    public:
                        PyCall( AbstractFunc* func );
                        
                        boost::python::object call( boost::python::object arg );
                        
                        boost::python::object callNoArg();
                };
                
            public:
                PyEntity( Entity* entity, Script* script = 0 );
                
                std::string getName();
                std::string getPath();
                
                bool hasFunc( const std::string& name );
                bool hasVar( const std::string& name );
                
                void func( boost::python::object func );
                
                boost::python::object getAttr( const std::string& name );
                void setAttr( const std::string& name, boost::python::object obj );
                
                static void defineClass();
                
                bool hasAspect( const std::string& name );
                void addAspect( const std::string& name );
                void removeAspect( const std::string& name );
                
                PyEntity getRoot();
                PyEntity getParent();
                
                bool hasChild( const std::string& name );
                PyEntity getChild( const std::string& name );
                PyEntity buildChild( const std::string& name );
                PyEntity loadChild( const std::string& filename );
                void removeChild( const std::string& name );
                
                void publish( const std::string& name, boost::python::object obj );
        };
        
    }
}

#endif
