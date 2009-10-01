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
    #ifdef FHE_THREAD
    Poco::ThreadPool Node::threadPool;
    #endif
    
    Node::Node() :
        m_refCount(0),
        m_parent(0)
    {
        addFunc("load_children",&Node::load_children,this);
        addFunc("load_vars",&Node::load_vars,this);
        addFunc("load_scripts",&Node::load_scripts,this);
    }
    
    Node::~Node()
    {
//         log("dtor %d",m_children.size());
    }
    
    void intrusive_ptr_add_ref(Node* p)
    {
        p->m_refCount++;
//         p->log("inc %d",p->m_refCount);
    }
    
    void intrusive_ptr_release(Node* p)
    {
//         p->log("dec %d",p->m_refCount-1);
        if (!--p->m_refCount)
        {
//             p->log("delete");
            delete p;
        }
    }
    
    void Node::init( const std::string& type, const std::string& name )
    {
        m_type = type;
        m_name = name;
        m_path = name;
//         log("ctor");
    }
    
    boost::python::object Node::defineClass()
    {
        return boost::python::class_<Node, boost::python::bases<VarMap,FuncMap>, boost::noncopyable >("Node", boost::python::no_init )
            .add_property("name",&Node::getName)
            .add_property("path",&Node::getPath)
            .def("__eq__",&Node::pyEquals)
            .def("attachToParent",&Node::pyAttachToParent)
            .def("detachFromParent",&Node::detachFromParent)
            .def("addChild",&Node::pyAddChild)
            .def("removeChild",&Node::pyRemoveChild)
            .def("clearChildren",&Node::clearChildren)
            .def("release",&Node::release)
            .def("getParent",&Node::pyGetParent)
            .def("hasChild",&Node::hasChild)
            .def("getChild",&Node::pyGetChild)
            .def("publish",&Node::publish)
            .def("loadChild",&Node::pyLoadChild)
            .def("buildChild",&Node::pyBuildChild)
            .def("getRoot",&Node::pyGetRoot)
            .def("log",&Node::pyLog)
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
            
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
            m_mainNamespace["Var"] = Var::defineClass();
            m_mainNamespace["VarMap"] = VarMap::defineClass();
            m_mainNamespace["FuncClosure"] = FuncMap::FuncClosure::defineClass();
            m_mainNamespace["FuncMap"] = FuncMap::defineClass();
            m_mainNamespace["Node"] = Node::defineClass();
        }
    }
    
    boost::python::dict Node::defaultNamespace()
    {
        initializePython();
        
        boost::python::dict ns;
        ns.update(m_mainNamespace);
        ns["self"] = toPy();
        return ns;
    }
    
    void Node::runScript( const std::string& filename )
    {
        runScript(filename,defaultNamespace());
    }
    
    void Node::runScript( const std::string& filename, boost::python::dict ns )
    {
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
        return tryEvalScript(s,defaultNamespace());
    }
    
    boost::python::object Node::tryEvalScript( const std::string& s, boost::python::dict ns )
    {
        try
        {
            Var val = Var::fromPy(boost::python::eval( s.c_str(), ns, ns ) );
            if ( !val.empty() )
            {
                return val.toPy();
            }
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Clear();
        }
        return boost::python::str(s);
    }
    
    boost::python::object Node::evalScript( const std::string& s )
    {
        return evalScript(s,defaultNamespace());
    }
    
    boost::python::object Node::evalScript( const std::string& s, boost::python::dict ns )
    {
        return boost::python::eval(s.c_str(),ns,ns);
    }
    
    void Node::execScript( const std::string& s )
    {
        execScript(s,defaultNamespace());
    }
    
    void Node::execScript( const std::string& s, boost::python::dict ns )
    {
        boost::python::exec(s.c_str(),ns,ns);
    }

    boost::python::object Node::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }
    
    NodePtr Node::fromPy( boost::python::object obj )
    {
        boost::python::extract<Node*> e(obj);
        return obj != boost::python::object() && e.check() ? e() : 0;
    }
    
    void Node::attachToParent( NodePtr parent )
    {
        if ( m_parent != parent )
        {
            detachFromParent();
            m_parent = parent.get();
            if ( m_parent )
            {
//                 log("attach to parent %s", m_parent->m_name.c_str());
                m_parent->addChild(this);
                if ( hasFunc<void,void>("on_attach") )
                {
                    call<void>("on_attach");
                }
            }
        }
    }
    
    void Node::pyAttachToParent( boost::python::object parent )
    {
        attachToParent( fromPy( parent ) );
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
    
    void Node::pyAddChild( boost::python::object child )
    {
        addChild(fromPy(child));
    }
    
    void Node::removeChild( NodePtr child )
    {
        if ( child && hasChild( child->m_name ) )
        {
            m_children.erase(child->m_name);
            child->detachFromParent();
        }
    }
    
    void Node::pyRemoveChild( boost::python::object child )
    {
        removeChild(fromPy(child));
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
//         log("release");
        detachFromParent();

        NodeList children;
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            children.push_back(i->second);
        }
        
        for ( NodeList::iterator i = children.begin(); i != children.end(); ++i )
        {
            (*i)->release();
        }
    }
    
    NodePtr Node::getParent()
    {
        return m_parent;
    }
    
    boost::python::object Node::pyGetParent()
    {
        return m_parent ? m_parent->toPy() : boost::python::object();
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
    
    boost::python::object Node::pyGetChild( const std::string& name )
    {
        NodePtr child = getChild(name);
        return child ? child->toPy() : boost::python::object();
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
/*            Var var = hasVarRaw(name) ? getVarRaw(name) : Var();
            if ( var.is<std::string>() )
            {
                std::string s = var.get<std::string>();
                if ( s[0] == '=' )
                {
                    return Var::fromPy(tryEvalScript(s.substr(1)));
                }
            }
            return var;*/
            return Var();
        }
    }
    
    #ifdef FHE_THREAD
    Node::PublishThread::PublishThread( NodePtr node, const std::string& cmd, const VarMap& args ) :
        m_node(node),
        m_cmd(cmd),
        m_args(args)
    {
    }
    
    void Node::PublishThread::run()
    {
        m_node->publish(m_cmd,m_args);
        m_event.set();
    }
    
    void Node::PublishThread::wait()
    {
        m_event.wait();
    }
    #endif
    
    void Node::publish( const std::string& cmd, const VarMap& args )
    {
        std::string msgName = "msg_" + cmd;
        if ( hasFunc<void,VarMap>(msgName) )
        {
            call<void,VarMap>(msgName,args);
        }
        
        #ifdef FHE_THREAD
        if ( !m_children.empty() )
        {
            std::vector<PublishThread*> threads;
        
            for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
            {
                if ( i->second->getVar<bool>("thread",true) )
                {
                    PublishThread* thread = new PublishThread(i->second,cmd,args);
                    Node::threadPool.start(*thread);
                    threads.push_back(thread);
                }
                else
                {
                    i->second->publish(cmd,args);
                }
            }
            
            for ( std::vector<PublishThread*>::iterator i = threads.begin(); i != threads.end(); ++i )
            {
                (*i)->wait();
                delete *i;
            }
        }
        #else
        for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
        {
            i->second->publish(cmd,args);
        }
        #endif
        
        std::string unmsgName = "unmsg_" + cmd;
        if ( hasFunc<void,VarMap>(unmsgName) )
        {
            call<void,VarMap>(unmsgName,args);
        }
    }
    
    bool Node::pyEquals( boost::python::object node )
    {
        return fromPy(node).get() == this;
    }

    void
    Node::log( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        printf("%s: %s\n", getPath().c_str(), buffer);
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
    
    boost::python::object Node::pyGetRoot()
    {
        return getRoot()->toPy();
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
    
    boost::python::object Node::pyLoadChild( const std::string& filename )
    {
        return loadChild(filename)->toPy();
    }
    
    boost::python::object Node::pyBuildChild( const std::string& type, const std::string& name )
    {
        NodePtr child = NodeFactory::instance().buildNode(type,name);
        addChild(child);
        return child->toPy();
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
    
    void Node::pyLog( const std::string& s )
    {
        log(s.c_str());
    }
}
