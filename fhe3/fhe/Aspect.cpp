#include "Aspect.h"

#include <stdexcept>

namespace fhe
{
    
    bool Aspect::m_pythonInitialized = false;
    boost::python::object Aspect::m_mainModule;
    boost::python::object Aspect::m_mainNamespace;
    
    Aspect::Aspect( const std::string& name ) :
        m_name(name)
    {
        addFunc("getName",&Aspect::getName,this);
    }
    
    boost::python::object Aspect::getAttr( const std::string& name )
    {
        if ( hasFuncName( name ) )
        {
            return boost::python::object(pyGetFunc(name));
        }
        else
        {
            throw std::runtime_error( "name error: " + name );
        }
    }
    
    std::string Aspect::getName()
    {
        return m_name;
    }
    
    void Aspect::setAttr( const std::string& name, boost::python::object obj )
    {
    }
    
    void Aspect::runScript( const std::string& filename )
    {
        try
        {
            initializePython();
            
            boost::python::dict ns;
            ns.update(m_mainNamespace);
            ns["self"] = toPy();
            
            boost::python::exec_file(filename.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            throw;
        }
    }
    
    boost::python::object Aspect::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }
    
    void Aspect::initializePython()
    {
        if ( !m_pythonInitialized )
        {
            m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            
            defineClass();
            PyCall::defineClass();
            FuncClosure::defineClass();
        }
    }
    
    boost::python::object Aspect::defineClass()
    {
        return boost::python::class_<Aspect, boost::noncopyable>("Aspect",boost::python::no_init)
            .def("__getattr__",&Aspect::getAttr)
            .def("__setattr__",&Aspect::setAttr)
            .def("func",&Aspect::func)
        ;
    }
    
    FuncMap::FuncClosure Aspect::func( boost::python::object tret, boost::python::object targ )
    {
        return FuncClosure(this,tret,targ);
    }
}
