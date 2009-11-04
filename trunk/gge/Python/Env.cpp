#include "Env.h"
#include "PyEntity.h"
#include "PyApp.h"

#include <gge/FileSystem.h>

namespace gge
{
    namespace Python
    {
        Env::Env()
        {
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            m_builtins = m_mainNamespace["__builtins__"].attr("__dict__");
            
            PyEntity::defineClass();
            PyApp::defineClass();
        }
        
        Env& Env::instance()
        {
            static Env env;
            return env;
        }
        
        boost::python::dict Env::defaultNamespace()
        {
            boost::python::dict ns;
            ns.update(m_mainNamespace);
            return ns;
        }
        
        void Env::runFile( const std::string& filename, boost::python::dict ns )
        {
            try
            {
                boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error running python file " + filename );
            }
        }
        
        void Env::exec( const std::string& script, boost::python::dict ns )
        {
            try
            {
                boost::python::exec( script.c_str(), ns, ns );
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error execing python script \"" + script + "\"" );
            }
        }
        
        boost::python::object Env::eval( const std::string& script, boost::python::dict ns )
        {
            try
            {
                return boost::python::eval( script.c_str(), ns, ns );
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error evaling python script \"" + script + "\"" );
            }
        }
        
        std::string Env::getType( boost::python::object obj )
        {
            return boost::python::extract<std::string>(m_builtins["type"](obj).attr("__name__"));
        }
        
        Var Env::convertToVar( boost::python::object obj )
        {
            if ( obj == boost::python::object() )
            {
                return Var();
            }
            else
            {
                std::string type = getType(obj);
                
                if ( type == "bool" )
                {
                    return Var::build<bool>(boost::python::extract<bool>(obj)());
                }
                else if ( type == "int" )
                {
                    return Var::build<int>(boost::python::extract<int>(obj)());
                }
                else if ( type == "float" )
                {
                    return Var::build<float>(boost::python::extract<float>(obj)());
                }
                else if ( type == "str" )
                {
                    return Var::build<std::string>(boost::python::extract<std::string>(obj)());
                }
                else
                {
                    throw std::runtime_error("unable to convert unknown python type " + type + " to var");
                }
            }
        }
        
        boost::python::object Env::convertFromVar( const Var& val )
        {
            if ( val.empty() )
            {
                return boost::python::object();
            }
            else 
            {
                if ( val.is<bool>() )
                {
                    return boost::python::object(val.get<bool>());
                }
                else if ( val.is<int>() )
                {
                    return boost::python::object(val.get<int>());
                }
                else if ( val.is<float>() )
                {
                    return boost::python::object(val.get<float>());
                }
                else if ( val.is<std::string>() )
                {
                    return boost::python::object(val.get<std::string>());
                }
                else
                {
                    throw std::runtime_error("unable to convert unknown c type to python");
                }
            }
        }
    }
}
