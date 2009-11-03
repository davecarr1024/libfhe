#include "PyEnv.h"

#include "Aspect.h"
#include "Entity.h"
#include "App.h"
#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"
#include "FileSystem.h"

namespace gge
{
    bool PyEnv::m_pythonInitialized = false;
    boost::python::object PyEnv::m_mainModule;
    boost::python::object PyEnv::m_mainNamespace;
    boost::python::object PyEnv::m_builtins;
    boost::python::object PyEnv::m_vec3;
    boost::python::object PyEnv::m_quat;
    boost::python::object PyEnv::m_vec2;
    boost::python::object PyEnv::m_rot;

    void PyEnv::initializePython()
    {
        if ( !m_pythonInitialized )
        {
            m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            m_builtins = m_mainNamespace["__builtins__"];
            
            Aspect::defineClass();
            Entity::defineClass();
            App::defineClass();
            m_vec3 = Vec3::defineClass();
            m_quat = Quat::defineClass();
            m_vec2 = Vec2::defineClass();
            m_rot = Rot::defineClass();
        }
    }

    boost::python::dict PyEnv::defaultNamespace()
    {
        initializePython();
        boost::python::dict ns;
        ns.update(m_mainNamespace);
        ns["Vec3"] = m_vec3;
        ns["Quat"] = m_quat;
        ns["Vec2"] = m_vec2;
        ns["Rot"] = m_rot;
        return ns;
    }
    
    void PyEnv::run( const std::string& filename, boost::python::dict ns )
    {
        initializePython();
        try
        {
            boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            throw;
        }
    }
    
    void PyEnv::exec( const std::string& s, boost::python::dict ns )
    {
        initializePython();
        try
        {
            boost::python::exec(s.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            throw;
        }
    }
    
    boost::python::object PyEnv::eval( const std::string& s, boost::python::dict ns )
    {
        initializePython();
        try
        {
            return boost::python::eval(s.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            throw;
        }
    }
    
    boost::python::object PyEnv::tryEval( const std::string& s, boost::python::dict ns )
    {
        initializePython();
        try
        {
            Var val = Var::fromPy(boost::python::eval(s.c_str(),ns,ns));
            return val.empty() ? boost::python::str(s) : val.toPy();
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Clear();
            return boost::python::str(s);
        }
    }
    
    std::string PyEnv::type( boost::python::object obj )
    {
        initializePython();
        return boost::python::extract<std::string>(m_builtins.attr("type")(obj).attr("__name__"));
    }
    
    std::string PyEnv::toString( boost::python::object obj )
    {
        initializePython();
        return boost::python::extract<std::string>(obj.attr("__repr__")());
    }
}
