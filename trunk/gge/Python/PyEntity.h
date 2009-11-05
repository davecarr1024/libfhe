#ifndef PYENTITY_H
#define PYENTITY_H

#include <gge/Entity.h>
#include <boost/python.hpp>

namespace gge
{
    namespace Python
    {
        
        class PyEntity
        {
            private:
                class FuncCall
                {
                    private:
                        AbstractFunc* m_func;
                        
                    public:
                        FuncCall( AbstractFunc* func );
                        
                        boost::python::object call( boost::python::object arg );
                        
                        boost::python::object callNoArg();
                };
            
                Entity* m_entity;
                
            public:
                PyEntity( Entity* entity );
                
                std::string getName();
                
                boost::python::object getAttr( const std::string& name );
                
                void setAttr( const std::string& name, boost::python::object val );
                
                bool hasAspect( const std::string& name );
                
                void addAspect( const std::string& name );
                
                void removeAspect( const std::string& name );
                
                static void defineClass();
        };
        
    }
}

#endif
