#include "Entity.h"

#include "FileSystem.h"

#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"

#include <sstream>

namespace gge
{
    
    Entity::Entity( const std::string& name ) :
        m_name(name),
        m_parent(0)
    {
    }
    
    void Entity::attachToParent( EntityPtr parent )
    {
        if ( parent.get() != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->addChild(this);
                callAll("on_attach");
            }
        }
    }
    
    void Entity::detachFromParent()
    {
        if ( m_parent )
        {
            EntityPtr parent = m_parent;
            m_parent = 0;
            parent->removeChild(this);
            callAll("on_detach");
        }
    }
    
    void Entity::addChild( EntityPtr child )
    {
        if ( child && !hasChild(child->getName()) )
        {
            m_children[child->getName()] = child;
            child->attachToParent(this);
        }
    }
    
    void Entity::removeChild( EntityPtr child )
    {
        if ( child && hasChild(child->getName()) )
        {
            m_children[child->getName()] = child;
            child->attachToParent(this);
        }
    }
    
    bool Entity::hasChild( const std::string& name )
    {
        return m_children.find(name) != m_children.end();
    }
    
    EntityPtr Entity::getChild( const std::string& name )
    {
        return hasChild(name) ? m_children[name] : 0;
    }
    
    EntityPtr Entity::buildChild( const std::string& name )
    {
        EntityPtr child( new Entity(name) );
        addChild(child);
        return child;
    }
    
    EntityPtr Entity::loadChild( const std::string& filename, const std::string& name )
    {
        EntityPtr child = buildChild(name);
        
        TiXmlDocument doc;
        
        if ( doc.LoadFile(FileSystem::instance().getFile(filename).c_str()) )
        {
            child->loadData(&doc);
        }
        else
        {
            throw std::runtime_error("unable to load entity file " + filename);
        }
        
        return child;
    }
    
    std::string Entity::getName()
    {
        return m_name;
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
        loadTag(h,"children");
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
                else if ( tag == "children" )
                {
                    loadChildren(e);
                }
            }
        }
    }
    
    void Entity::loadChildren( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("child").ToElement(); e; e = e->NextSiblingElement("child") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            buildChild(name)->loadData(e);
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

    AbstractFunc* Entity::getFunc( const std::string& name ) const
    {
        for ( AspectMap::const_iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                return i->second->getFunc(name);
            }
        }
        return 0;
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
    
    void Entity::publish( const std::string& name, const Var& arg )
    {
        callAll("msg_" + name,arg);
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(name,arg);
        }
        callAll("unmsg_" + name,arg);
    }
}
