#ifndef PYENV_H
#define PYENV_H

#include <boost/python.hpp>

namespace fhe
{
    class PyEnv
    {
        private:
            PyEnv();
            
            PyEnv( const PyEnv& pe );
            void operator=( const PyEnv& pe );
            
            boost::python::object m_mainModule, m_mainNamespace, m_builtins;
            
        public:
            static PyEnv& instance();
            
            boost::python::dict defaultNamespace();
            
            void runFile( const std::string& filename, boost::python::dict ns );
            void exec( const std::string& script, boost::python::dict ns );
            boost::python::object eval( const std::string& script, boost::python::dict ns );
            
            std::string getType( boost::python::object obj );
            std::string toString( boost::python::object obj );
    };
}

#endif
