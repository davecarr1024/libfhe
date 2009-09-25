#include "Node.h"
#include <stdexcept>

namespace fhe
{
    
    FHE_NODE_IMPL(Node);
    
    boost::python::object Node::m_mainNamespace;
    boost::python::object Node::m_mainModule;
    boost::python::object Node::m_addFunc;
    bool Node::m_pythonInitialized = false;
    
    Node::Node() :
        m_refCount(0),
        m_parent(0)
    {
    }
    
    void Node::init( const std::string& type, const std::string& name )
    {
        m_type = type;
        m_name = name;
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
        ;
    }
    
    void Node::initializePython()
    {
        if ( !Node::m_pythonInitialized )
        {
            Node::m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = m_mainModule.attr("__dict__");
            
            boost::python::dict ns;
            boost::python::exec_file("Core/PyNode.py",ns,ns);
            m_addFunc = ns["addFunc"];
            
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
            boost::python::exec_file( filename.c_str(), ns, ns );
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
        
        try
        {
            return boost::python::eval( s.c_str(), ns, ns );
        }
        catch ( boost::python::error_already_set const& )
        {
            return boost::python::object(s);
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
        self.attr("func") = m_addFunc(self);
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
    
    void Node::onGetVar( const std::string& name )
    {
        std::string getName = "get_" + name;
        if ( hasFunc<Var,void>(getName) )
        {
            m_vars[name] = call<Var>(getName);
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
}
