#ifndef PYTHON_ENV_H
#define PYTHON_ENV_H

#include <boost/python.hpp>
#include <gge/Var.h>

namespace gge
{
    namespace Python
    {
        
        class Env
        {
            private:
                boost::python::object m_mainModule, m_mainNamespace, m_builtins;
                
                Env();
                
                Env( const Env& env ) {}
                void operator=( const Env& env ){}
                
            public:
                static Env& instance();
                
                boost::python::dict defaultNamespace();
                
                void runFile( const std::string& filename, boost::python::dict ns );
                
                void exec( const std::string& script, boost::python::dict ns );
                
                boost::python::object eval( const std::string& script, boost::python::dict ns );
                
                std::string getType( boost::python::object obj );
                
                Var convertToVar( boost::python::object obj );
                
                boost::python::object convertFromVar( const Var& val );
        };
        
    }
}

#endif
