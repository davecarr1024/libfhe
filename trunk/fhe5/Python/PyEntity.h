#ifndef PYENTITY_H
#define PYENTITY_H

#include <boost/python.hpp>
#include <fhe/Entity.h>

namespace fhe
{
    namespace Python
    {
        
        class PyEntity
        {
            private:
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
                PyEntity( Entity* entity );
                
                std::string getName();
                std::string getPath();
                
                bool hasFunc( const std::string& name );
                bool hasVar( const std::string& name );
                
                boost::python::object getAttr( const std::string& name );
                void setAttr( const std::string& name, boost::python::object obj );
                
                static void defineClass();
        };
        
    }
}

#endif
