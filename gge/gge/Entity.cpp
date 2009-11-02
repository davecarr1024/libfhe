#include "Entity.h"
#include "App.h"

#include <sstream>

namespace gge
{
    
    Entity::Entity( const std::string& name ) :
        m_name(name),
        m_app(0)
    {
    }
    
    std::string Entity::getName()
    {
        return m_name;
    }
    
    App* Entity::getApp()
    {
        return m_app;
    }
    
    void Entity::addAspect( AspectPtr aspect )
    {
        if ( aspect && !hasAspect(aspect->getName()) )
        {
            m_aspects[aspect->getName()] = aspect;
            aspect->attachToEntity(this);
        }
    }
    
    void Entity::removeAspect( AspectPtr aspect )
    {
        if ( aspect && hasAspect(aspect->getName()) )
        {
            m_aspects.erase(aspect->getName());
            aspect->detachFromEntity();
        }
    }
    
    bool Entity::hasAspect( const std::string& name )
    {
        return m_aspects.find(name) != m_aspects.end();
    }
    
    AspectPtr Entity::getAspect( const std::string& name )
    {
        return hasAspect(name) ? m_aspects[name] : 0;
    }
    
    AspectPtr Entity::buildAspect( const std::string& name )
    {
        if ( !hasAspect(name) )
        {
            addAspect(AspectFactory::instance().build(name));
        }
        return getAspect(name);
    }
    
    void Entity::attachToApp( App* app )
    {
        if ( m_app != app )
        {
            detachFromApp();
            m_app = app;
            if ( m_app )
            {
                m_app->addEntity(this);
            }
        }
    }
    
    void Entity::detachFromApp()
    {
        if ( m_app )
        {
            App* app = m_app;
            m_app = 0;
            app->removeEntity(this);
        }
    }
    
    Var Entity::onGetVar( const std::string& name ) const
    {
        std::string get = "get_" + name;
        if ( hasFunc<Var,void>(get) )
        {
            return const_cast<Entity*>(this)->call<Var>(get);
        }
        else
        {
            return Var();
        }
    }
    
    void Entity::onSetVar( const std::string& name, const Var& val )
    {
        callAll<void,Var>("set_" + name,val);
    }
    
    bool Entity::onHasVar( const std::string& name ) const
    {
        return hasFunc<Var,void>("get_" + name);
    }
    
    void Entity::loadData( TiXmlHandle h )
    {
        loadTag(h,"vars");
        loadTag(h,"aspects");
    }
    
    void Entity::loadTag( TiXmlHandle h, const std::string& tag )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            if ( e->Value() == tag )
            {
                if ( tag == "vars" )
                {
                    loadVars(e);
                }
                else if ( tag == "aspects" )
                {
                    loadAspects(e);
                }
            }
        }
    }
    
    void Entity::loadVars( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("var").ToElement(); e; e = e->NextSiblingElement("var") )
        {
            const char* cname = e->Attribute("name");
            assert(cname);
            const char* ctype = e->Attribute("type");
            assert(ctype);
            std::string name(cname), type(ctype), value( e->GetText() );
            std::istringstream ins(value);
            if ( type == "bool" )
            {
                bool b;
                ins >> b;
                setVar<bool>(name,b);
            }
            else if ( type == "int" )
            {
                int i;
                ins >> i;
                setVar<int>(name,i);
            }
            else if ( type == "float" )
            {
                float f;
                ins >> f;
                setVar<float>(name,f);
            }
            else if ( type == "string" )
            {
                setVar<std::string>(name,value);
            }
            else
            {
                throw std::runtime_error("unknown var type " + type);
            }
        }
    }
    
    void Entity::loadAspects( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("aspect").ToElement(); e; e = e->NextSiblingElement("aspect") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            buildAspect(name)->loadData(e);
        }
    }
    
    boost::python::object Entity::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }
    
    boost::python::object Entity::defineClass()
    {
        return boost::python::class_<Entity,boost::noncopyable>("Entity",boost::python::no_init)
            .add_property("name",&Entity::getName)
            .def("hasVar",&Entity::hasVarName)
            .def("hasFunc",&Entity::hasFuncName)
            .def("hasAspect",&Entity::hasAspect)
            .def("getAspect",&Entity::getAspect)
            .def("buildAspect",&Entity::buildAspect)
            .def("__getattr__",&Entity::getAttr)
            .def("__setattr__",&Entity::setAttr)
        ;
    }
    
    boost::python::object Entity::getAttr( const std::string& name )
    {
        if ( name == "app" )
        {
            return m_app ? m_app->toPy() : boost::python::object();
        }
        else if ( hasVarName( name ) )
        {
            return getRawVar(name).toPy();
        }
        else
        {
            for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
            {
                if ( i->second->hasFuncName(name) )
                {
                    return i->second->pyGetFunc(name);
                }
            }
        }
        return boost::python::object();
    }
    
    void Entity::setAttr( const std::string& name, boost::python::object val )
    {
        setRawVar(name,Var::fromPy(val));
    }
    
    bool Entity::hasFuncName( const std::string& name )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFuncName(name) )
            {
                return true;
            }
        }
        return false;
    }
    
    boost::python::object Entity::pyGetAspect( const std::string& name )
    {
        AspectPtr aspect = getAspect(name);
        return aspect ? aspect->toPy() : boost::python::object();
    }
    
    boost::python::object Entity::pyBuildAspect( const std::string& name )
    {
        return buildAspect(name)->toPy();
    }
}
