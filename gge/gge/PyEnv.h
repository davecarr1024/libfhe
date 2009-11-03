#ifndef PYENV_H
#define PYENV_H

#include <boost/python.hpp>

namespace gge
{
    
    class PyEnv
    {
        private:
            static bool m_pythonInitialized;
            static boost::python::object m_mainModule, m_mainNamespace, m_vec3, m_quat, m_vec2, m_rot, m_builtins;
            
            static void initializePython();

        public:
            static boost::python::dict defaultNamespace();

            static void run( const std::string& name, boost::python::dict ns );
            
            static void exec( const std::string& s, boost::python::dict ns );
            
            static boost::python::object eval( const std::string& s, boost::python::dict ns );
            
            static boost::python::object tryEval( const std::string& s, boost::python::dict ns );
            
            static std::string type( boost::python::object obj );
            
            static std::string toString( boost::python::object obj );
    };
    
}

#endif
