#include "Aspect.h"
#include "Entity.h"
#include "App.h"
#include "FileSystem.h"

#include <cstdarg>

namespace gge
{
    
    GGE_TO_PYTHON_CONVERTER( AspectPtr, obj ? obj->toPy() : boost::python::object() );
    GGE_FROM_PYTHON_CONVERTER( AspectPtr, AspectPtr(boost::python::extract<Aspect*>(obj)) );

    GGE_ASPECT(Aspect);
    
    bool Aspect::m_pythonInitialized = false;
    boost::python::object Aspect::m_mainModule;
    boost::python::object Aspect::m_mainNamespace;

    Aspect::Aspect() :
        m_entity(0)
    {
    }
    
    Aspect::~Aspect()
    {
    }
    
    void Aspect::init( const std::string& name )
    {
        m_name = name;
    }
    
    std::string Aspect::getName()
    {
        return m_name;
    }
    
    std::string Aspect::getPath()
    {
        return (m_entity ? m_entity->getName() : "<no entity>") + "." + m_name;
    }
    
    Entity* Aspect::getEntity()
    {
        return m_entity;
    }
    
    void Aspect::log( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        printf("%s: %s\n", getPath().c_str(), buffer);
    }

    void Aspect::error( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        throw std::runtime_error( getPath() + ": ERROR: " + buffer );
    }

    void Aspect::attachToEntity( Entity* entity )
    {
        if ( entity != m_entity )
        {
            detachFromEntity();
            m_entity = entity;
            if ( m_entity )
            {
                m_entity->addAspect(this);
                if ( hasFunc<void,void>("on_attach") )
                {
                    call<void>("on_attach");
                }
            }
        }
    }
    
    void Aspect::detachFromEntity()
    {
        if ( m_entity )
        {
            Entity* entity = m_entity;
            m_entity = 0;
            entity->removeAspect(this);
            if ( hasFunc<void,void>("on_detach") )
            {
                call<void>("on_detach");
            }
        }
    }
    
    void Aspect::loadData( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            std::string load = std::string("load_") + e->Value();
            if ( hasFunc<void,TiXmlHandle>(load) )
            {
                call<void,TiXmlHandle>(load,e);
            }
        }
    }
    
    boost::python::object Aspect::getAttr( const std::string& name )
    {
        if ( name == "entity" )
        {
            EntityPtr entity( getEntity() );
            return entity ? entity->toPy() : boost::python::object();
        }
        else if ( hasFuncName(name) )
        {
            return pyGetFunc(name);
        }
        else
        {
            return boost::python::object();
        }
    }
    
    void Aspect::setAttr( const std::string& name, boost::python::object val )
    {
        error("state can only be stored in entities");
    }
    
    boost::python::object Aspect::defineClass()
    {
        FuncMap::defineClass();
        return boost::python::class_<Aspect,boost::noncopyable>("Aspect",boost::python::no_init)
            .def("func",&Aspect::pyAddFunc)
            .def("hasFunc",&Aspect::hasFuncName)
            .def("__getattr__",&Aspect::getAttr)
            .def("__setattr__",&Aspect::setAttr)
        ;
    }
    
    boost::python::object Aspect::toPy()
    {
        initializePython();
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
            Entity::defineClass();
            App::defineClass();
        }
    }
    
    boost::python::dict Aspect::defaultNamespace()
    {
        initializePython();
        boost::python::dict ns;
        ns.update(m_mainNamespace);
        return ns;
    }
    
    boost::python::dict Aspect::selfNamespace()
    {
        boost::python::dict ns = defaultNamespace();
        ns["self"] = toPy();
        return ns;
    }
    
    void Aspect::runScript( const std::string& filename )
    {
        initializePython();
        boost::python::dict ns = selfNamespace();
        try
        {
            boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            PyErr_Print();
            error("running script %s",filename.c_str());
        }
    }
    
    void Aspect::execScript( const std::string& s, boost::python::dict ns )
    {
        initializePython();
        try
        {
            boost::python::exec(s.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            log("error executing script %s",s.c_str());
            PyErr_Print();
            PyErr_Clear();
        }
    }
    
    boost::python::object Aspect::evalScript( const std::string& s, boost::python::dict ns )
    {
        initializePython();
        try
        {
            return boost::python::eval(s.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set )
        {
            log("error evaling script %s",s.c_str());
            PyErr_Print();
            PyErr_Clear();
            return boost::python::object();
        }
    }
}
