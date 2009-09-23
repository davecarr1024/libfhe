#include "Node.h"
#include "NodeFactory.h"
#include "PyNode.h"
#include "VarMap.h"
#include <cstdio>
#include <sstream>
#include <cassert>
#include <cstring>
#include <cstdarg>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/python.hpp>

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

namespace fhe
{
    
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
    }
    
    Node::~Node()
    {
        clearFuncs();
    }
    
    void intrusive_ptr_add_ref(Node* p)
    {
        p->m_refCount++;
    }
    
    void intrusive_ptr_release(Node* p)
    {
        if (!--p->m_refCount)
        {
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
                if ( hasFunc<void,void>("on_attach") )
                {
                    callFunc<void>("on_attach");
                }
            }
            updatePath();
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
    Node::load( const std::string& path )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( path.c_str() ) )
        {
            return Node::load( &doc );
        }
        else
        {
            throw std::runtime_error("can't load node file " + path);
        }
    }
    
    NodePtr
    Node::load( TiXmlHandle h )
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
        
        if ( node )
        {
            node->doLoad( h );
        }
        
        return node;
    }
    
    void
    Node::doLoad( TiXmlHandle h )
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
                doLoad( &doc );
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
            NodePtr child = Node::load( e );
            if ( child )
            {
                addChild( child );
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
        for ( std::map<std::string, IFuncWrapper*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            if ( i->first.substr(0,5) == "save_" && hasFunc<TiXmlElement*,void>(i->first) )
            {
                node->LinkEndChild( callFunc<TiXmlElement*>(i->first) );
            }
        }
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
        TiXmlElement* vars = new TiXmlElement("vars");
        
        for ( std::map<std::string,Var>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            TiXmlElement* var = new TiXmlElement("var");
            
            std::string name( i->first );
            
            var->SetAttribute( "name", name.c_str() );
            
            std::ostringstream outs;
            if ( hasVar<bool>(name) )
            {
                var->SetAttribute( "type", "bool" );
                outs << (int)getVar<bool>(name);
            }
            else if ( hasVar<int>(name) )
            {
                var->SetAttribute( "type", "int" );
                outs << getVar<int>(name);
            }
            else if ( hasVar<float>(name) )
            {
                var->SetAttribute( "type", "float" );
                outs << getVar<float>(name);
            }
            else if ( hasVar<std::string>(name) )
            {
                var->SetAttribute( "type", "string" );
                outs << getVar<std::string>(name);
            }
            
            var->LinkEndChild( new TiXmlText( outs.str().c_str() ) );
            vars->LinkEndChild( var );
        }
        
        return vars;
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
            
            m_mainNamespace["VarMap"] = VarMap::defineClass();
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
            
            m_nodeClass = PyNode::defineClass(); 
        }
    }
    
    void
    Node::runScript( const std::string& filename )
    {
        initializePython();
        
        boost::python::dict ns;
        ns.update( m_mainNamespace );

        ns["self"] = PyNode::create( this );
        
        try
        {
            exec_file(filename.c_str(), ns, ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            error("running script %s", filename.c_str());
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
        printf("%s: %s\n", m_path.c_str(), buffer);
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

    void Node::removeFunc( const std::string& name )
    {
        if ( m_funcs.find( name ) != m_funcs.end() )
        {
            delete m_funcs[name];
            m_funcs.erase(name);
        }
    }

    void Node::clearFuncs()
    {
        for ( std::map<std::string, IFuncWrapper*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            delete i->second;
        }
        m_funcs.clear();
    }
    
    bool Node::pyHasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    boost::python::object Node::pyGetVar( const std::string& name )
    {
        assert( pyHasVar( name ) );
        return m_vars[name].toPy();
    }
    
    boost::python::object Node::pyGetVarDef( const std::string& name, boost::python::object def )
    {
        if ( pyHasVar( name ) )
        {
            return pyGetVar( name );
        }
        else
        {
            return def;
        }
    }
    
    void Node::pySetVar( const std::string& name, boost::python::object val )
    {
        m_vars[name] = Var::fromPy( val );
    }
}
