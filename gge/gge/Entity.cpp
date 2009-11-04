#include "Entity.h"
#include "App.h"

#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"

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
        return hasFunc(get) ? const_cast<Entity*>(this)->call(get) : Var();
    }
    
    void Entity::onSetVar( const std::string& name, const Var& val )
    {
        callAll("set_" + name,val);
    }
    
    bool Entity::onHasVar( const std::string& name ) const
    {
        return hasFunc("get_" + name);
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
            const char* cname = e->Attribute("name"), *ctype = e->Attribute("type");
            assert(cname);
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
            else if ( type == "Vec2" )
            {
                float x, y;
                ins >> x >> y;
                setVar<Vec2>(name,Vec2(x,y));
            }
            else if ( type == "Vec3" )
            {
                float x, y, z;
                ins >> x >> y >> z;
                setVar<Vec3>(name,Vec3(x,y,z));
            }
            else if ( type == "Rot" )
            {
                float a;
                ins >> a;
                setVar<Rot>(name,Rot(a));
            }
            else if ( type == "Quat" )
            {
                float x, y, z, a;
                ins >> x >> y >> z >> a;
                setVar<Quat>(name,Quat(Vec3(x,y,z),a));
            }
            else
            {
                throw std::runtime_error( "unable to set var " + name + " to unknown type " + type );
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
    
    bool Entity::hasFunc( const std::string& name ) const
    {
        for ( AspectMap::const_iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                return true;
            }
        }
        return false;
    }

    Var Entity::call( const std::string& name, const Var& arg )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                return i->second->call(name,arg);
            }
        }
        throw std::runtime_error("entity " + getName() + " has no func " + name );
    }
    
    void Entity::callAll( const std::string& name, const Var& arg )
    {
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                i->second->call(name,arg);
            }
        }
    }
}
