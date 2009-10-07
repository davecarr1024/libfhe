#include "Node.h"
#include "NodeFactory.h"
#include "PyNode.h"
#include "VarMap.h"
#include "FileSystem.h"

#include "math/fheMath.h"
#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

#include <cstdio>
#include <sstream>
#include <cassert>
#include <cstring>
#include <cstdarg>
#include <stdexcept>

#include <boost/bind.hpp>
#include <boost/python.hpp>

namespace fhe
{
    
    int g_count = 0;
    
    NODE_IMPL(Node);
    
    bool Node::m_pythonInitialized = false;
    
    boost::python::object Node::m_mainModule;
    boost::python::object Node::m_mainNamespace;
    boost::python::object Node::m_nodeClass;
    
    std::map<std::string, int> Node::m_nameCount;

    Node::Node( const std::string& name, const std::string& type ) :
        m_type( type ),
        m_refCount(0),
        m_parent(0)
    {
        if ( m_nameCount.find( name ) == m_nameCount.end() )
        {
            m_name = name;
            m_nameCount[name] = 1;
        }
        else
        {
            std::ostringstream outs;
            outs << name << "_" << ++m_nameCount[name];
            m_name = outs.str();
        }
        
        addFunc("load_vars",&Node::load_vars,this);
        addFunc("load_includes",&Node::load_includes,this);
        addFunc("load_children",&Node::load_children,this);
        addFunc("load_scripts",&Node::load_scripts,this);
        
        addFunc("save_vars",&Node::save_vars,this);
        addFunc("save_children",&Node::save_children,this);
        addFunc("save_name",&Node::save_name,this);
        addFunc("save_type",&Node::save_type,this);
        
        updatePath();
        
        log("ctor %d", ++g_count);
    }
    
    Node::~Node()
    {
        log("dtor %d", --g_count);
    }
    
    void intrusive_ptr_add_ref(Node* p)
    {
        p->m_refCount++;
        p->log("inc %d",p->m_refCount);
    }
    
    void intrusive_ptr_release(Node* p)
    {
        p->log("dec %d",p->m_refCount-1);
        if (!--p->m_refCount)
        {
            p->log("delete");
            delete p;
        }
    }

    void
    Node::release()
    {
        detachFromParent();
        removeAllChildren();
    }
    
    std::string
    Node::getType()
    {
        return m_type;
    }
    
    std::string
    Node::getPath()
    {
        return m_path;
    }
    
    std::string
    Node::getName()
    {
        return m_name;
    }
    
    void
    Node::updatePath()
    {
        if ( m_parent && m_parent->m_parent )
        {
            m_path = m_parent->m_path + "/" + m_name;
        }
        else if ( m_parent )
        {
            m_path = "/" + m_name;
        }
        else
        {
            m_path = "/";
        }
    }
    
    void
    Node::attachToParent( NodePtr parent )
    {
        if ( parent != m_parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->addChild( this );
                updatePath();
                if ( hasFunc<void,void>("on_attach") )
                {
                    callFunc<void>("on_attach");
                }
            }
        }
    }
    
    void
    Node::detachFromParent()
    {
        if ( m_parent )
        {
            NodePtr parent = m_parent;
            m_parent = 0;
            parent->removeChild( this );
            if ( hasFunc<void,void>("on_detach") )
            {
                callFunc<void>("on_detach");
            }
            updatePath();
        }
    }
    
    bool
    Node::hasChild( const std::string& name )
    {
        return m_children.find( name ) != m_children.end();
    }
    
    bool
    Node::hasChild( NodePtr child )
    {
        return child && hasChild( child->m_name );
    }
    
    void
    Node::addChild( const std::string& path )
    {
        NodePtr child = Node::create(path);
        assert(child);
        addChild(child);
        child->load(path);
    }
    
    void
    Node::addChild( TiXmlHandle h )
    {
        NodePtr child = Node::create(h);
        assert(child);
        addChild(child);
        child->load(h);
    }
    
    void
    Node::addChild( NodePtr child )
    {
        if ( child && !hasChild( child ) )
        {
            m_children[child->m_name] = child;
            child->attachToParent( this );
        }
    }
    
    void
    Node::removeChild( NodePtr child )
    {
        if ( child && hasChild( child ) )
        {
            m_children.erase( child->m_name );
            child->detachFromParent();
        }
    }
    
    void
    Node::removeAllChildren()
    {
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->release();
        }
        m_children.clear();
    }
    
    NodePtr
    Node::getChild( const std::string& name )
    {
        return hasChild( name ) ? m_children[name] : 0;
    }
    
    NodePtr
    Node::getParent()
    {
        return m_parent;
    }
    
    NodePtr
    Node::getRoot()
    {
        return m_parent ? m_parent->getRoot() : this;
    }
    
    NodePtr
    Node::getLocalNode( const std::string& path )
    {
        if ( path == "." )
        {
            return this;
        }
        else if ( path == ".." )
        {
            assert( m_parent );
            return m_parent;
        }
        else
        {
            NodePtr child = getChild( path );
            assert( child );
            return child;
        }
    }
    
    NodePtr
    Node::getNode( const std::string& path )
    {
        if ( path == "/" )
        {
            return getRoot();
        }
        else if ( path[0] == '/' )
        {
            return getRoot()->getNode( path.substr( 1 ) );
        }
        else
        {
            size_t pos = path.find("/");
            if ( pos == std::string::npos )
            {
                return getLocalNode( path );
            }
            else
            {
                return getLocalNode( path.substr( 0, pos ) )->getNode( path.substr( pos + 1 ) );
            }
        }
    }
    
    NodePtr
    Node::create( const std::string& path )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( path.c_str() ) )
        {
            return Node::create( &doc );
        }
        else
        {
            throw std::runtime_error("can't load node file " + path);
        }
    }
    
    NodePtr
    Node::create( TiXmlHandle h )
    {
        std::string type = "Node", name;
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            if ( !strcmp(e->Value(),"type") )
            {
                type = e->GetText();
            }
            else if ( !strcmp(e->Value(), "name" ) )
            {
                name = e->GetText();
            }
        }
        
        if ( name.empty() )
        {
            name = type;
        }
        
        NodePtr node = NodeFactory::instance().buildNode(type,name);
        if ( !node )
        {
            throw std::runtime_error("couldn't create a node of type " + type );
        }
        return node;
    }
    
    void
    Node::load( const std::string& path )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( path.c_str() ) )
        {
            load( &doc );
        }
        else
        {
            error("can't load file %s", path.c_str());
        }
    }
    
    void
    Node::load( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            std::string tag( e->Value() ), funcName = "load_" + tag;
            
            if ( tag != "type" && tag != "name" )
            {
                if ( !hasFunc<void,TiXmlElement*>( funcName ) )
                {
                    throw std::runtime_error("ERROR: unknown file tag " + tag );
                }
                else
                {
                    callFunc<void,TiXmlElement*>( funcName, e );
                }
            }
        }
    }
    
    void
    Node::load_vars( TiXmlElement* elem )
    {
        for ( TiXmlElement* e = elem->FirstChildElement("var"); e; e = e->NextSiblingElement("var") )
        {
            const char *nameAttr = e->Attribute("name"),
                       *typeAttr = e->Attribute("type"),
                       *valueAttr = e->GetText();
                       
            if ( !nameAttr || !typeAttr || !valueAttr )
            {
                throw std::runtime_error("ERROR: invalid var tag\n");
            }
            else
            {
                std::istringstream ins(valueAttr);
                std::string name(nameAttr), type(typeAttr);
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
                    setVar<std::string>(name,valueAttr);
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
                    setVar<Rot>(name,Rot(Math::radians(a)));
                }
                else if ( type == "Quat" )
                {
                    float x, y, z, a;
                    ins >> x >> y >> z >> a;
                    setVar<Quat>(name,Quat(Vec3(x,y,z),Math::radians(a)));
                }
                else
                {
                    throw std::runtime_error("can't load var " + name + ", is unknown type " + type );
                }
            }
        }
    }
    
    void
    Node::load_includes( TiXmlElement* elem )
    {
        for ( TiXmlElement* e = elem->FirstChildElement("include"); e; e = e->NextSiblingElement("include") )
        {
            TiXmlDocument doc;
            if ( doc.LoadFile( e->GetText() ) )
            {
                load( &doc );
            }
            else
            {
                throw std::runtime_error( std::string("unable to load included file ") + e->GetText() );
            }
        }
    }
    
    void
    Node::load_children( TiXmlElement* elem )
    {
        for ( TiXmlElement* e = elem->FirstChildElement("child"); e; e = e->NextSiblingElement("child") )
        {
            NodePtr child = Node::create( e );
            if ( child )
            {
                addChild( child );
                child->load(e);
            }
            else
            {
                throw std::runtime_error("ERROR: failed to create child\n");
            }
        }
    }
    
    void
    Node::load_scripts( TiXmlElement* elem )
    {
        for ( TiXmlElement* e = elem->FirstChildElement("script"); e; e = e->NextSiblingElement("script") )
        {
            runScript( e->GetText() );
        }
    }
    
    void
    Node::saveInto( TiXmlNode* node )
    {
        //TODO
    }
    
    void
    Node::save( const std::string& filename )
    {
        TiXmlDocument doc;
        saveInto( &doc );
        doc.SaveFile(filename.c_str());
    }
    
    TiXmlElement*
    Node::save_vars()
    {
        //TODO
    }
    
    TiXmlElement*
    Node::save_children()
    {
        TiXmlElement* children = new TiXmlElement("children");
        
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            TiXmlElement* child = new TiXmlElement("child");
            i->second->saveInto(child);
            children->LinkEndChild(child);
        }
        
        return children;
    }
    
    TiXmlElement*
    Node::save_name()
    {
        TiXmlElement* name = new TiXmlElement("name");
        name->LinkEndChild( new TiXmlText( m_name.c_str() ) );
        return name;
    }
    
    TiXmlElement*
    Node::save_type()
    {
        TiXmlElement* type = new TiXmlElement("type");
        type->LinkEndChild( new TiXmlText( m_type.c_str() ) );
        return type;
    }
    
    void
    Node::initializePython()
    {
        if ( !m_pythonInitialized )
        {
            m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = m_mainModule.attr("__dict__");

            m_mainNamespace["FuncClosure"] = PyNode::FuncClosure::defineClass();
            m_mainNamespace["Var"] = Var::defineClass();
            m_mainNamespace["VarMap"] = VarMap::defineClass();
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
            
            m_nodeClass = PyNode::defineClass(); 
        }
    }
    
    void
    Node::runScript( const std::string& _filename )
    {
        initializePython();
        
        std::string filename = FileSystem::instance().getFile( _filename );
        
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        log("add self");

        ns["self"] = PyNode::create( this );

        log("run script");

        try
        {
            exec_file(filename.c_str(), ns, ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            error("running script %s", filename.c_str());
        }
        
        log("/run script");
    }
    
    boost::python::object
    Node::evalScript( const std::string& s )
    {
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        ns["self"] = PyNode::create( this );
        
        try
        {
            return boost::python::eval(s.c_str(), ns, ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            error("evaling script: %s", s.c_str());
        }
    }
    
    boost::python::object
    Node::tryEvalScript( const std::string& s )
    {
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        ns["self"] = PyNode::create( this );
        
        try
        {
            return boost::python::eval(s.c_str(), ns, ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Clear();
            return boost::python::str(s);
        }
    }
    
    std::vector<std::string>
    Node::getChildNames()
    {
        std::vector<std::string> names;
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            names.push_back( i->first );
        }
        return names;
    }

    void
    Node::log( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        printf("%s: %s\n", m_name.c_str(), buffer);
    }

    void
    Node::error( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        throw std::runtime_error( m_path + ": ERROR: " + buffer );
    }
    
    void
    Node::onSetVar( const std::string& name, const Var& val )
    {
        std::string setName = "set_" + name;
        if ( hasFunc<void,Var>(setName) )
        {
            callFunc<void,Var>(setName,val);
        }
    }
    
    Var
    Node::onGetVar( const std::string& name )
    {
        std::string getName = "get_" + name;
        if ( hasFunc<Var,void>(getName) )
        {
            return callFunc<Var>(getName);
        }
        return Var();
    }
}
