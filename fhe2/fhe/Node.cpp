#include "Node.h"
#include "FileSystem.h"

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

#include <stdexcept>
#include <cstdarg>
#include <cstdio>

namespace fhe
{
    
    FHE_NODE_IMPL(Node);
    
    boost::python::object Node::m_mainModule;
    boost::python::dict Node::m_mainNamespace;
    bool Node::m_pythonInitialized = false;
    
    Node::Node() :
        m_refCount(0),
        m_parent(0)
    {
        addFunc("load_children",&Node::load_children,this);
        addFunc("load_vars",&Node::load_vars,this);
        addFunc("load_scripts",&Node::load_scripts,this);
    }
    
    void Node::init( const std::string& type, const std::string& name )
    {
        m_type = type;
        m_name = name;
        m_path = name;
    }
    
    boost::python::object Node::defineClass()
    {
        return boost::python::class_<Node, NodePtr, boost::python::bases<VarMap,FuncMap> >("Node", boost::python::no_init )
            .def("__eq__",&Node::pyEquals)
            .def("attachToParent",&Node::attachToParent)
            .def("detachFromParent",&Node::detachFromParent)
            .def("addChild",&Node::addChild)
            .def("removeChild",&Node::removeChild)
            .def("clearChildren",&Node::clearChildren)
            .def("release",&Node::release)
            .def("getParent",&Node::getParent)
            .def("hasChild",&Node::hasChild)
            .def("getChild",&Node::getChild)
            .def("runScript",&Node::runScript)
            .def("publish",&Node::publish)
            .def("buildNode",&Node::pyBuildNode)
            .def("loadChild",&Node::loadChild)
        ;
    }
    
    void Node::initializePython()
    {
        if ( !Node::m_pythonInitialized )
        {
            Node::m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            
            boost::python::dict ns;
            boost::python::exec_file("fhe/PyNode.py",ns,ns);
            m_mainNamespace.update(ns);
            
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
            m_mainNamespace["Var"] = Var::defineClass();
            m_mainNamespace["VarMap"] = VarMap::defineClass();
            m_mainNamespace["FuncMap"] = FuncMap::defineClass();
            m_mainNamespace["Node"] = Node::defineClass();
        }
    }
    
    void Node::runScript( const std::string& filename )
    {
        initializePython();
        
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        ns["self"] = toPy();
        
        try
        {
            boost::python::exec_file( FileSystem::instance().getFile(filename).c_str(), ns, ns );
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            throw std::runtime_error( "error running python file " + filename );
        }
    }
    
    boost::python::object Node::tryEvalScript( const std::string& s )
    {
        initializePython();
        
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        ns["self"] = toPy();
        
        return m_mainNamespace["tryEval"](s,ns);
        
        try
        {
            return boost::python::eval( s.c_str(), ns, ns );
        }
        catch ( boost::python::error_already_set const& )
        {
            return boost::python::str(s);
        }
    }
    
    boost::python::object Node::evalScript( const std::string& s )
    {
        initializePython();
        
        boost::python::dict ns;
        ns.update( m_mainNamespace );
        
        ns["self"] = toPy();
        
        try
        {
            return boost::python::eval( s.c_str(), ns, ns );
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            throw std::runtime_error( "error running python script: " + s );
        }
    }

    boost::python::object Node::toPy()
    {
        boost::python::object self(this);
        self.attr("func") = m_mainNamespace["addFunc"](self);
        return self;
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
    
    void Node::attachToParent( NodePtr parent )
    {
        if ( m_parent != parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
                m_parent->addChild(this);
                if ( hasFunc<void,void>("on_attach") )
                {
                    call<void>("on_attach");
                }
            }
        }
    }
    
    void Node::detachFromParent()
    {
        if ( m_parent )
        {
            NodePtr parent = m_parent;
            m_parent = 0;
            parent->removeChild(this);
            if ( hasFunc<void,void>("on_detach") )
            {
                call<void>("on_detach");
            }
        }
    }
    
    void Node::addChild( NodePtr child )
    {
        if ( child && !hasChild( child->m_name ) )
        {
            m_children[child->m_name] = child;
            child->attachToParent(this);
        }
    }
    
    void Node::removeChild( NodePtr child )
    {
        if ( child && hasChild( child->m_name ) )
        {
            m_children.erase(child->m_name);
            child->detachFromParent();
        }
    }
    
    void Node::clearChildren()
    {
        NodeList children;
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            children.push_back(i->second);
        }
        
        for ( NodeList::iterator i = children.begin(); i != children.end(); ++i )
        {
            removeChild(*i);
        }
    }
    
    void Node::release()
    {
        detachFromParent();
        clearChildren();
    }
    
    NodePtr Node::getParent()
    {
        return m_parent;
    }
    
    bool Node::hasChild( const std::string& name )
    {
        return m_children.find(name) != m_children.end();
    }
    
    NodePtr Node::getChild( const std::string& name )
    {
        assert(hasChild(name));
        return m_children[name];
    }
    
    void Node::onSetVar( const std::string& name, const Var& val )
    {
        std::string setName = "set_" + name;
        if ( hasFunc<void,Var>(setName) )
        {
            call<void,Var>(setName,val);
        }
    }
    
    Var Node::onGetVar( const std::string& name )
    {
        std::string getName = "get_" + name;
        if ( hasFunc<Var,void>(getName) )
        {
            return  call<Var>(getName);
        }
        else
        {
            Var var = hasVarRaw(name) ? getVarRaw(name) : Var();
            if ( var.is<std::string>() )
            {
                std::string s = var.get<std::string>();
                if ( s[0] == '=' )
                {
                    return Var::fromPy(tryEvalScript(s.substr(1)));
                }
            }
            return var;
        }
    }
    
    void Node::publish( const std::string& cmd, const VarMap& args )
    {
        std::string msgName = "msg_" + cmd;
        if ( hasFunc<void,VarMap>(msgName) )
        {
            call<void,VarMap>(msgName,args);
        }
        
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(cmd,args);
        }
        
        std::string unmsgName = "unmsg_" + cmd;
        if ( hasFunc<void,VarMap>(unmsgName) )
        {
            call<void,VarMap>(unmsgName,args);
        }
    }
    
    boost::python::object Node::pyBuildNode( const std::string& type, const std::string& name )
    {
        return NodeFactory::instance().buildNode(type,name)->toPy();
    }
    
    bool Node::pyEquals( NodePtr node )
    {
        return node == this;
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
    
    std::string Node::getName()
    {
        return m_name;
    }
    
    std::string Node::getPath()
    {
        return m_path;
    }
    
    NodePtr Node::getRoot()
    {
        return m_parent ? m_parent->getRoot() : this;
    }
    
    NodePtr Node::loadChild( const std::string& filename )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( FileSystem::instance().getFile(filename).c_str() ) )
        {
            return loadChildData( &doc );
        }
        else
        {
            error( "unable to load node file %s", filename.c_str() );
        }
    }
    
    NodePtr Node::loadChildData( TiXmlHandle h )
    {
        NodePtr child = createChild( h );
        assert(child);
        addChild(child);
        fillChild(child,h);
        return child;
    }
    
    NodePtr Node::createChild( TiXmlHandle h )
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
            error("couldn't create a node of type %s", type.c_str() );
        }
        return node;
    }
    
    void Node::fillChild( NodePtr node, TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            std::string tag(e->Value()), loadName = "load_" + tag;
            if ( node->hasFunc<void,TiXmlHandle>(loadName) )
            {
                node->call<void,TiXmlHandle>(loadName,TiXmlHandle(e));
            }
            else if ( tag != "type" && tag != "name" )
            {
                error("unknown file tag %s", tag.c_str());
            }
        }
    }
    
    void Node::load_children( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("child").ToElement(); e; e = e->NextSiblingElement("child") )
        {
            loadChildData(e);
        }
    }
    
    void Node::load_vars( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("var").ToElement(); e; e = e->NextSiblingElement("var") )
        {
            const char *name = e->Attribute("name");
            assert(name);
            boost::python::object val = tryEvalScript(e->GetText());
            std::string type = boost::python::extract<std::string>(val.attr("__class__").attr("__name__"));
            pySetVar(name,val);
        }
    }
    
    void Node::load_scripts( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("script").ToElement(); e; e = e->NextSiblingElement("script") )
        {
            runScript(e->GetText());
        }
    }
}
