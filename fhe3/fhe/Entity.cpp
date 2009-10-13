#include "Entity.h"
#include "AspectFactory.h"
#include <sstream>

namespace fhe
{
    
    std::map<std::string,int> Entity::m_nameCounts;
    
    FHE_TO_CONVERTER(EntityPtr,boost::python::object(boost::python::ptr(obj.get())));
    FHE_FROM_CONVERTER(EntityPtr,obj == boost::python::object() ? 0 : EntityPtr(boost::python::extract<Entity*>(obj)(),true));
    
    EntityPtr Entity::root( new Entity("root") );
    
    Entity::Entity( const std::string& name ) :
        m_parent(0)
    {
        if ( Entity::m_nameCounts.find(name) == Entity::m_nameCounts.end() )
        {
            m_name = name;
            Entity::m_nameCounts[name] = 1;
        }
        else
        {
            std::ostringstream outs;
            outs << name << "_" << ++Entity::m_nameCounts[name];
            m_name = outs.str();
        }
        updatePath();
    }
    
    Entity::~Entity()
    {
    }
    
    std::string Entity::getName()
    {
        return m_name;
    }
    
    std::string Entity::getPath()
    {
        return m_path;
    }
    
    void Entity::updatePath()
    {
        if ( m_parent )
        {
            if ( m_parent->m_parent )
            {
                m_path = m_parent->m_path + "/" + m_name;
            }
            else
            {
                m_path = "/" + m_name;
            }
        }
        else
        {
            m_path = "/";
        }
    }
    
    void Entity::attachToParent( EntityPtr parent )
    {
        if ( parent != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            updatePath();
            if ( m_parent )
            {
                m_parent->addChild(EntityPtr(this,true));
            }
        }
    }
    
    void Entity::detachFromParent()
    {
        if ( m_parent )
        {
            Entity* parent = m_parent;
            m_parent = 0;
            updatePath();
            parent->removeChild(EntityPtr(this,true));
        }
    }
    
    void Entity::addChild( EntityPtr child )
    {
        if ( child && !hasChild(child->m_name) )
        {
            m_children[child->m_name] = child;
            child->attachToParent(EntityPtr(this,true));
        }
    }
    
    void Entity::removeChild( EntityPtr child )
    {
        if ( child && hasChild(child->m_name) )
        {
            m_children.erase(child->m_name);
            child->detachFromParent();
        }
    }
    
    bool Entity::hasChild( const std::string& name )
    {
        return m_children.find(name) != m_children.end();
    }
    
    EntityPtr Entity::getChild( const std::string& name )
    {
        EntityMap::iterator i = m_children.find(name);
        return i == m_children.end() ? 0 : i->second;
    }
    
    EntityPtr Entity::getParent()
    {
        return EntityPtr(m_parent,true);
    }
    
    AspectPtr Entity::addAspect( const std::string& name )
    {
        if ( !hasAspect(name) )
        {
            AspectPtr aspect = AspectFactory::instance().buildAspect(name);
            assert(aspect);
            m_aspects[aspect->getName()] = aspect;
            aspect->setEntity(EntityPtr(this,true));
            return aspect;
        }
        else
        {
            return getAspect(name);
        }
    }
    
    void Entity::removeAspect( const std::string& name )
    {
        if ( hasAspect(name) )
        {
            AspectPtr aspect = m_aspects[name];
            m_aspects.erase(name);
            aspect->setEntity(0);
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
    
    Var Entity::onGetVar( const std::string& name )
    {
        Var var;
        std::string getName = "get_" + name;
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc<Var,void>(getName) )
            {
                var = i->second->call<Var>(getName);
                if ( !var.empty() )
                {
                    return var;
                }
            }
        }
        return var;
    }
    
    void Entity::onSetVar( const std::string& name, const Var& var )
    {
        std::string setName = "set_" + name;
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc<void,Var>(setName) )
            {
                i->second->call<void,Var>(setName,var);
            }
        }
    }
    
    void Entity::publish( const std::string& cmd, const VarMap& args )
    {
        std::string msgName = "msg_" + cmd;
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc<void,VarMap>(msgName) )
            {
                i->second->call<void,VarMap>(msgName,args);
            }
        }
        
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(cmd,args);
        }

        std::string unmsgName = "unmsg_" + cmd;
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFunc<void,VarMap>(unmsgName) )
            {
                i->second->call<void,VarMap>(unmsgName,args);
            }
        }
    }
    
    void Entity::pyPublish( const std::string& cmd, boost::python::dict args )
    {
        publish(cmd,VarMap::fromPy(args));
    }

    boost::python::object Entity::getAttr( const std::string& name )
    {
        if ( pyHasVar(name) )
        {
            return pyGetVar(name);
        }
        
        for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
        {
            if ( i->second->hasFuncName(name) )
            {
                return boost::python::object(i->second->pyGetFunc(name));
            }
        }
        throw std::runtime_error( "name error: " + name );
    }
    
    void Entity::setAttr( const std::string& name, boost::python::object val )
    {
        pySetVar(name,val);
    }
    
    boost::python::object Entity::defineClass()
    {
        return boost::python::class_<Entity,boost::noncopyable>("Entity",boost::python::no_init)
            .def("__getattr__",&Entity::getAttr)
            .def("__setattr__",&Entity::setAttr)
            .def("hasVar",&Entity::pyHasVar)
            .def("getVar",&Entity::pyGetVar)
            .def("getVar",&Entity::pyGetVarDef)
            .def("setVar",&Entity::pySetVar)
            .def("publish",&Entity::pyPublish)
            .add_property("name",&Entity::getName)
            .add_property("path",&Entity::getPath)
        ;
    }
    
}