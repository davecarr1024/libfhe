#include <fhe/core/PyEnv.h>
#include <fhe/core/Val.h>
#include <fhe/core/PyNode.h>
#include <fhe/core/FileSystem.h>
#include <fhe/core/Vec.h>
#include <fhe/core/Rot.h>
#include <fhe/core/Mat.h>

BOOST_PYTHON_MODULE( fhe )
{
    fhe::PyNode::defineClass();
    fhe::Vec2::defineClass();
    fhe::Vec3::defineClass();
    fhe::Rot2::defineClass();
    fhe::Rot3::defineClass();
    fhe::Mat2::defineClass();
    fhe::Mat3::defineClass();
}

namespace fhe
{
    PyEnv::PyEnv()
    {
        Py_Initialize();
        
        m_mainModule = boost::python::import("__main__");
        m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
        m_builtins = m_mainNamespace["__builtins__"].attr("__dict__");
        
        m_mainNamespace["Node"] = PyNode::defineClass();
        m_mainNamespace["Vec2"] = Vec2::defineClass();
        m_mainNamespace["Vec3"] = Vec3::defineClass();
        m_mainNamespace["Rot2"] = Rot2::defineClass();
        m_mainNamespace["Rot3"] = Rot3::defineClass();
        m_mainNamespace["Mat2"] = Mat2::defineClass();
        m_mainNamespace["Mat3"] = Mat3::defineClass();
    }
    
    PyEnv& PyEnv::instance()
    {
        static PyEnv pe;
        return pe;
    }
    
    boost::python::dict PyEnv::defaultNamespace()
    {
        boost::python::dict ns;
        ns.update(m_mainNamespace);
        return ns;
    }
    
    void PyEnv::runFile( const std::string& filename, boost::python::dict ns )
    {
        try
        {
            boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            FHE_ERROR( "error running python file %s", filename.c_str() );
        }
    }
    
    void PyEnv::exec( const std::string& script, boost::python::dict ns )
    {
        try
        {
            boost::python::exec(script.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            FHE_ERROR( "error execing python script %s", script.c_str() );
        }
    }
    
    boost::python::object PyEnv::eval( const std::string& script, boost::python::dict ns )
    {
        try
        {
            return boost::python::eval(script.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            PyErr_Clear();
            FHE_ERROR( "error evaling python script %s", script.c_str() );
        }
    }
    
    std::string PyEnv::getType( boost::python::object obj )                                    
    {                                                                                        
        return boost::python::extract<std::string>(m_builtins["type"](obj).attr("__name__"));
    }                                                                                        
    
    std::string PyEnv::toString( boost::python::object obj )
    {
        return boost::python::extract<std::string>( m_builtins["str"](obj) );
    }
}