#include "Entity.h"
#include "AspectFactory.h"
#include "FileSystem.h"
#include <stdexcept>
#include <sstream>

namespace fhe
{
    std::map<std::string,int> Entity::m_nameCounts;
    
    Entity::Entity( const std::string& name ) :
        m_refCount(0),
        m_parent(0),
        m_name(Entity::makeName(name))
    {
    }
    
    std::string Entity::makeName( const std::string& name )
    {
        if ( Entity::m_nameCounts.find(name) == Entity::m_nameCounts.end() )
        {
            Entity::m_nameCounts[name] = 1;
            return name;
        }
        else
        {
            std::ostringstream outs;
            outs << name << "_" << ++Entity::m_nameCounts[name];
            return outs.str();
        }
    }
    
    void Entity::incRef()
    {
        ++m_refCount;
    }
    
    bool Entity::decRef()
    {
        return --m_refCount;
    }
    
    std::string Entity::getName()
    {
        return m_name;
    }
    
    std::string Entity::getPath()
    {
        if ( m_parent && m_parent->m_parent )
        {
            return m_parent->getPath() + "/" + getName();
        }
        else if ( m_parent )
        {
            return "/" + getName();
        }
        else
        {
            return "/";
        }
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
                _call("on_attach");
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
            _call("on_detach");
        }
    }
    
    EntityPtr Entity::getParent()
    {
        return m_parent;
    }
    
    EntityPtr Entity::getRoot()
    {
        return m_parent ? m_parent->getRoot() : EntityPtr(this);
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
            m_children.erase(child->getName());
            child->detachFromParent();
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
            addAspect(AspectFactory::instance().buildAspect(name));
        }
        return getAspect(name);
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
    
    Var Entity::_call( const std::string& name, const Var& arg )
    {
        Var res;
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc(name) )
            {
                res = i->second->_call(name,arg);
            }
        }
        return res;
    }
    
    void Entity::_publish( const std::string& name, const Var& arg )
    {
        _call("msg_" + name,arg);
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->_publish(name,arg);
        }
        _call("unmsg_" + name,arg);
    }
    
    EntityPtr Entity::loadChild( const std::string& filename )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile(FileSystem::instance().getFile(filename).c_str()) )
        {
            EntityPtr child = buildChild(filename.substr(0,filename.find(".")).substr(filename.rfind("/")+1));
            child->loadData(&doc);
            return child;
        }
        else
        {
            throw std::runtime_error("unable to load entity file " + filename);
        }
    }
    
    void Entity::loadData( TiXmlHandle h )
    {
        loadTag(h,"vars");
        loadTag(h,"aspects");
        loadTag(h,"vars");
        loadTag(h,"children");
    }
    
    void Entity::loadTag( TiXmlHandle h, const std::string& tag )
    {
        for ( TiXmlElement* e = h.FirstChildElement(tag.c_str()).ToElement(); e; e = e->NextSiblingElement(tag.c_str()) )
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
    
    void Entity::loadVars( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("var").ToElement(); e; e = e->NextSiblingElement("var") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            _setVar(name,Var::load(e));
        }
    }
    
    void Entity::loadAspects( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("aspect").ToElement(); e; e = e->NextSiblingElement("aspect") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            buildAspect(name)->load(e);
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
    
    Var Entity::onGetVar( const std::string& name ) const
    {
        return const_cast<Entity*>(this)->_call("get_" + name);
    }
    
    void Entity::onSetVar( const std::string& name, const Var& val )
    {
        const_cast<Entity*>(this)->_call("set_" + name,val);
    }
    
    bool Entity::onHasVar( const std::string& name ) const
    {
        return hasFunc("get_" + name);
    }
    
    std::vector<std::string> Entity::getChildNames()
    {
        std::vector<std::string> names;
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            names.push_back(i->first);
        }
        return names;
    }
}
