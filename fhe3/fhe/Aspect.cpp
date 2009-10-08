#include "Aspect.h"
#include "Entity.h"

#include <stdexcept>

namespace fhe
{
    
    FHE_TO_CONVERTER(AspectPtr,boost::python::object(boost::python::ptr(obj.get())));
    FHE_FROM_CONVERTER(AspectPtr,obj == boost::python::object() ? 0 : AspectPtr(boost::python::extract<Aspect*>(obj)(),true));
    
    bool Aspect::m_pythonInitialized = false;
    boost::python::object Aspect::m_mainModule;
    boost::python::object Aspect::m_mainNamespace;
    
    Aspect::Aspect( const std::string& name ) :
        m_name(name),
        m_entity(0)
    {
        addFunc("getName",&Aspect::getName,this);
        addFunc("getEntity",&Aspect::getEntity,this);
    }
    
    std::string Aspect::getName()
    {
        return m_name;
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
            Entity::defineClass();
        }
    }
    
    boost::python::object Aspect::defineClass()
    {
        return boost::python::class_<Aspect, boost::noncopyable>("Aspect",boost::python::no_init)
            .def("__getattr__",&Aspect::getAttr)
            .def("func",&Aspect::func)
        ;
    }
    
    FuncMap::FuncClosure Aspect::func( boost::python::object tret, boost::python::object targ )
    {
        return FuncClosure(this,tret,targ);
    }
    
    void Aspect::setEntity( EntityPtr entity )
    {
        m_entity = entity.get();
    }
    
    EntityPtr Aspect::getEntity()
    {
        return m_entity ? EntityPtr(m_entity,true) : 0;
    }
}
