#include "Aspect.h"
#include "PyEnv.h"

#include <cstdarg>

namespace gge
{
    
    GGE_TO_PYTHON_CONVERTER( AspectPtr, obj ? obj->toPy() : boost::python::object() );
    GGE_FROM_PYTHON_CONVERTER( AspectPtr, AspectPtr(boost::python::extract<Aspect*>(obj)) );

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
        return boost::python::object(boost::python::ptr(this));
    }
}
