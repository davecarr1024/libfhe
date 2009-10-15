#include "Entity.h"
#include "AspectFactory.h"
#include "FileSystem.h"

#include <sstream>
#include <cstdarg>
#include <cstdio>

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
            .def("loadChild",&Entity::pyLoadChild)
            .def("buildChild",&Entity::pyBuildChild)
            .def("hasChild",&Entity::hasChild)
            .def("getChild",&Entity::pyGetChild)
            .def("getParent",&Entity::pyGetParent)
            .def("hasFunc",&Entity::pyHasFunc)
            .def("getRoot",&Entity::pyGetRoot)
            .add_property("name",&Entity::getName)
            .add_property("path",&Entity::getPath)
        ;
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
                callAll<void>("on_attach");
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
            callAll<void>("on_detach");
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
    
    boost::python::object Entity::pyGetChild( const std::string& name )
    {
        EntityPtr child( getChild(name) );
        return child ? child->toPy() : boost::python::object();
    }
    
    boost::python::object Entity::pyGetParent()
    {
        EntityPtr parent = getParent();
        return parent ? parent->toPy() : boost::python::object();
    }
    
    AspectPtr Entity::addAspect( const std::string& name )
    {
        if ( !hasAspect(name) )
        {
            AspectPtr aspect = AspectFactory::instance().buildAspect(name);
            assert(aspect);
            m_aspects[aspect->getName()] = aspect;
            aspect->setEntity(EntityPtr(this,true));
            if ( aspect->hasFunc<void,void>("on_attach") )
            {
                aspect->call<void>("on_attach");
            }
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
            if ( aspect->hasFunc<void,void>("on_detach") )
            {
                aspect->call<void>("on_detach");
            }
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
        if ( hasFunc<Var,void>(getName) )
        {
            var = call<Var>(getName);
        }
        return var;
    }
    
    void Entity::onSetVar( const std::string& name, const Var& var )
    {
        callAll<void,Var>("set_" + name,var);
    }
    
    void Entity::publish( const std::string& cmd, const VarMap& args )
    {
        callAll<void,VarMap>("msg_" + cmd,args);
        
        for ( EntityMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(cmd,args);
        }

        callAll<void,VarMap>("unmsg_" + cmd,args);
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
        error("name error: %s", name.c_str());
    }
    
    void Entity::setAttr( const std::string& name, boost::python::object val )
    {
        pySetVar(name,val);
    }
    
    EntityPtr Entity::loadChild( const std::string& filename )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( FileSystem::instance().getFile(filename).c_str() ) )
        {
            return loadChildData( &doc );
        }
        else
        {
            error("unable to load file %s", filename.c_str());
        }
    }
    
    boost::python::object Entity::pyLoadChild( const std::string& filename )
    {
        return loadChild(filename)->toPy();
    }
    
    EntityPtr Entity::loadChildData( TiXmlHandle h )
    {
        std::string name = "Ent";
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            if ( !strcmp(e->Value(),"name") )
            {
                name = e->GetText();
            }
        }
        
        EntityPtr child( new Entity(name) );
        
        addChild(child);
        child->load(h);
        return child;
    }
    
    void Entity::load( TiXmlHandle h )
    {
        loadTag(h,"vars");
        loadTag(h,"aspects");
        loadTag(h,"vars");
        loadTag(h,"children");
    }
    
    void Entity::loadTag( TiXmlHandle h, const std::string& tag )
    {
        printf("%s load %s\n",getPath().c_str(),tag.c_str());
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
    
    void Entity::loadVars( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("var").ToElement(); e; e = e->NextSiblingElement("var") )
        {
            const char *name = e->Attribute("name");
            assert(name);
            boost::python::dict ns = Aspect::emptyNamespace();
            ns["entity"] = toPy();
            boost::python::object val = Var::fromPy(Aspect::tryEvalScript(e->GetText(),ns)).toPy();
            pySetVar(name,val);
        }
    }
    
    void Entity::loadChildren( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("child").ToElement(); e; e = e->NextSiblingElement("child") )
        {
            loadChildData(e);
        }
    }
    
    void Entity::loadAspects( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("aspect").ToElement(); e; e = e->NextSiblingElement("aspect") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            AspectPtr aspect( addAspect(name) );
            assert(aspect);
            aspect->load(e);
        }
    }
    
    EntityPtr Entity::buildChild( const std::string& name )
    {
        EntityPtr child( new Entity(name) );
        addChild(child);
        return child;
    }
    
    boost::python::object Entity::pyBuildChild( const std::string& name )
    {
        return buildChild(name)->toPy();
    }
    
    boost::python::object Entity::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }

    void Entity::error( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        throw std::runtime_error( m_path + ": ERROR: " + buffer );
    }
    
    bool Entity::pyHasFunc( const std::string& name )
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
    
    EntityPtr Entity::getRoot()
    {
        return m_parent ? m_parent->getRoot() : EntityPtr(this,true);
    }
    
    boost::python::object Entity::pyGetRoot()
    {
        return getRoot()->toPy();
    }
}
