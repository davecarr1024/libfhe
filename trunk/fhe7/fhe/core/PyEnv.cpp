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
    
    fhe::Vec2d::defineClass();
    fhe::Vec3d::defineClass();
    fhe::Vec2i::defineClass();
    fhe::Vec3i::defineClass();
    
    fhe::Rot2d::defineClass();
    fhe::Rot3d::defineClass();
    fhe::Rot2i::defineClass();
    fhe::Rot3i::defineClass();
    
    fhe::Mat2d::defineClass();
    fhe::Mat3d::defineClass();
    fhe::Mat2i::defineClass();
    fhe::Mat3i::defineClass();
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
        m_mainNamespace["Vec2d"] = Vec2d::defineClass();
        m_mainNamespace["Vec3d"] = Vec3d::defineClass();
        m_mainNamespace["Vec2i"] = Vec2i::defineClass();
        m_mainNamespace["Vec3i"] = Vec3i::defineClass();
        m_mainNamespace["Rot2d"] = Rot2d::defineClass();
        m_mainNamespace["Rot3d"] = Rot3d::defineClass();
        m_mainNamespace["Rot2i"] = Rot2i::defineClass();
        m_mainNamespace["Rot3i"] = Rot3i::defineClass();
        m_mainNamespace["Mat2d"] = Mat2d::defineClass();
        m_mainNamespace["Mat3d"] = Mat3d::defineClass();
        m_mainNamespace["Mat2i"] = Mat2i::defineClass();
        m_mainNamespace["Mat3i"] = Mat3i::defineClass();
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
            return boost::python::object();
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
