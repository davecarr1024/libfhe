#include "PyNode.h"
#include "NodeFactory.h"
#include "math/Vec2.h"
#include <cassert>
#include <cstdio>

namespace fhe
{
    
    bool PyNode::m_initialized = false;
    
    boost::python::object PyNode::m_addFunc;
    
    void PyNode::initialize()
    {
        if ( !m_initialized )
        {
            m_initialized = true;
            
            boost::python::dict ns;

            boost::python::exec_file("fhe/PyNode.py",ns,ns);
            
            m_addFunc = ns["addFunc"];
        }
    }
    
    boost::python::object PyNode::defineClass()
    {
        return boost::python::class_<PyNode>( "Node", boost::python::no_init )
            .add_property("name", &PyNode::getName )
            .add_property("type", &PyNode::getType )
            .add_property("path", &PyNode::getPath )
            .def( "__eq__", &PyNode::equals )
            .def( "release", &PyNode::release )
            .def( "getParent", &PyNode::getParent )
            .def( "hasChild", &PyNode::hasChild )
            .def( "getChild", &PyNode::getChild )
            .def( "getRoot", &PyNode::getRoot )
            .def( "getNode", &PyNode::getNode )
            .def( "call", &PyNode::callFunc )
            .def( "publish", &PyNode::publish )
            .def( "setVar", &PyNode::setVar )
            .def( "getVar", &PyNode::getVar )
            .def( "getVar", &PyNode::getVarDef )
            .def( "hasVar", &PyNode::hasVar )
            .def( "_addFunc", &PyNode::addFunc )
            .def( "attachToParent", &PyNode::attachToParent )
            .def( "detachFromParent", &PyNode::detachFromParent )
            .def( "addChild", &PyNode::addChild )
            .def( "removeChild", &PyNode::removeChild )
            .def( "getChildNames", &PyNode::getChildNames )
            .def( "buildNode", &PyNode::buildNode )
            .def( "log", &PyNode::log )
        ;  
    }
    
    boost::python::object PyNode::create( NodePtr node )
    {
        if ( node )
        {
            boost::python::object pyNode(new PyNode( node ));
            
            pyNode.attr("func") = m_addFunc(pyNode);
            
            return pyNode;
        }
        else
        {
            return boost::python::object();
        }
    }
    
    PyNode::PyNode( NodePtr node ) :
        m_node( node )
    {
        assert( m_node );
        initialize();
    }
    
    void PyNode::release()
    {
        m_node->release();
    }
    
    std::string PyNode::getName()
    {
        return m_node->getName();
    }
    
    std::string PyNode::getType()
    {
        return m_node->getType();
    }
    
    std::string PyNode::getPath()
    {
        return m_node->getPath();
    }
    
    boost::python::object PyNode::getParent()
    {
        NodePtr parent = m_node->getParent();
        if ( parent )
        {
            return create( parent );
        }
        else
        {
            return boost::python::object();
        }
    }
    
    bool PyNode::hasChild( const std::string& name )
    {
        return m_node->hasChild( name );
    }
    
    boost::python::object PyNode::getChild( const std::string& name )
    {
        if ( hasChild( name ) )
        {
            return create( m_node->getChild( name ) );
        }
        else
        {
            return boost::python::object();
        }
    }
    
    boost::python::object PyNode::getRoot()
    {
        return create( m_node->getRoot() );
    }
    
    boost::python::object PyNode::getNode( const std::string& path )
    {
        NodePtr node = m_node->getNode( path );
        if ( node )
        {
            return create( node );
        }
        else
        {
            return boost::python::object();
        }
    }
    
    boost::python::object PyNode::getChildNames()
    {
        boost::python::list names;
        std::vector<std::string> cnames = m_node->getChildNames();
        for ( std::vector<std::string>::iterator i = cnames.begin(); i != cnames.end(); ++i )
        {
            names.append(*i);
        }
        return names;
    }
    
    void PyNode::attachToParent( PyNode* parent )
    {
        if ( parent )
        {
            m_node->attachToParent( parent->m_node );
        }
    }
    
    void PyNode::detachFromParent()
    {
        m_node->detachFromParent();
    }
    
    void PyNode::addChild( PyNode* child )
    {
        if ( child )
        {
            m_node->addChild( child->m_node );
        }
    }
    
    void PyNode::removeChild( PyNode* child )
    {
        if ( child )
        {
            m_node->removeChild( child->m_node );
        }
    }
    
    std::string PyNode::getPyType( boost::python::object obj )
    {
        return boost::python::extract<std::string>( obj.attr("__class__").attr("__name__") );
    }
    
    bool PyNode::hasFunc( const std::string& name )
    {
        return m_node->m_funcs.find(name) != m_node->m_funcs.end();
    }
    
    boost::python::object PyNode::callFuncNoArg( const std::string& name )
    {
        if ( m_node->hasFunc<void,void>( name ) )
        {
            m_node->callFunc<void>( name );
            return boost::python::object();
        }
        else if ( m_node->hasFunc<bool,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<bool>( name ));
        }
        else if ( m_node->hasFunc<int,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<int>( name ));
        }
        else if ( m_node->hasFunc<float,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<float>( name ));
        }
        else if ( m_node->hasFunc<std::string,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<std::string>( name ));
        }
        else if ( m_node->hasFunc<VarMap,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<VarMap>( name ));
        }
        else if ( m_node->hasFunc<Vec2,void>( name ) )
        {
            return boost::python::object( m_node->callFunc<Vec2>( name ));
        }
        else
        {
            m_node->error( "unable to find func %s for python call", name.c_str() );
        }
    }
    
    template <class TArg>
    boost::python::object PyNode::callFuncWithArg( const std::string& name, const TArg& arg )
    {
        if ( m_node->hasFunc<void,TArg>( name ) )
        {
            m_node->callFunc<void,TArg>( name, arg );
            return boost::python::object();
        }
        else if ( m_node->hasFunc<bool,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<bool,TArg>( name, arg ) );
        }
        else if ( m_node->hasFunc<int,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<int,TArg>( name, arg ) );
        }
        else if ( m_node->hasFunc<float,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<float,TArg>( name, arg ) );
        }
        else if ( m_node->hasFunc<std::string,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<std::string,TArg>( name, arg ) );
        }
        else if ( m_node->hasFunc<VarMap,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<VarMap,TArg>( name, arg ) );
        }
        else if ( m_node->hasFunc<Vec2,TArg>( name ) )
        {
            return boost::python::object( m_node->callFunc<Vec2,TArg>( name, arg ) );
        }
        else
        {
            m_node->error( "unable to fnid func %s for python call", name.c_str() );
        }
    }
    
    boost::python::object PyNode::callFunc( const std::string& name, boost::python::object arg )
    {
        if ( arg == boost::python::object() )
        {
            return callFuncNoArg( name );
        }
        
        std::string type = getPyType( arg );
        if ( type == "bool" )
        {
            return callFuncWithArg<bool>( name, boost::python::extract<bool>(arg) );
        }
        else if ( type == "int" )
        {
            return callFuncWithArg<int>( name, boost::python::extract<int>(arg) );
        }
        else if ( type == "float" )
        {
            return callFuncWithArg<float>( name, boost::python::extract<float>(arg) );
        }
        else if ( type == "str" )
        {
            return callFuncWithArg<std::string>( name, boost::python::extract<std::string>(arg) );
        }
        else if ( type == "VarMap" )
        {
            return callFuncWithArg<VarMap>( name, boost::python::extract<VarMap>(arg) );
        }
        else if ( type == "Vec2" )
        {
            return callFuncWithArg<Vec2>( name, boost::python::extract<Vec2>(arg) );
        }
        else
        {
            m_node->error( "unable to call func %s with arg type %s", name.c_str(), type.c_str() );
        }
    }
    
    boost::python::object PyNode::getVar( const std::string& name )
    {
        return getVarDef( name, boost::python::object() );
    }
    
    boost::python::object PyNode::getVarDef( const std::string& name, boost::python::object def )
    {
        if ( m_node->hasVar<bool>( name ) )
        {
            return boost::python::object( m_node->getVar<bool>( name ) );
        }
        else if ( m_node->hasVar<int>( name ) )
        {
            return boost::python::object( m_node->getVar<int>( name ) );
        }
        else if ( m_node->hasVar<float>( name ) )
        {
            return boost::python::object( m_node->getVar<float>( name ) );
        }
        else if ( m_node->hasVar<std::string>( name ) )
        {
            return boost::python::object( m_node->getVar<std::string>( name ) );
        }
        else if ( m_node->hasVar<VarMap>( name ) )
        {
            return boost::python::object( m_node->getVar<VarMap>( name ) );
        }
        else if ( m_node->hasVar<Vec2>( name ) )
        {
            return boost::python::object( m_node->getVar<Vec2>( name ) );
        }
        else
        {
            return def;
        }
    }
    
    void PyNode::setVar( const std::string& name, boost::python::object val )
    {
        std::string type = getPyType( val );
        
        if ( type == "bool" )
        {
            m_node->setVar<bool>( name, boost::python::extract<bool>( val ) );
        }
        else if ( type == "int" )
        {
            m_node->setVar<int>( name, boost::python::extract<int>( val ) );
        }
        else if ( type == "float" )
        {
            m_node->setVar<float>( name, boost::python::extract<float>( val ) );
        }
        else if ( type == "str" )
        {
            m_node->setVar<std::string>( name, boost::python::extract<std::string>( val ) );
        }
        else if ( type == "VarMap" )
        {
            m_node->setVar<VarMap>( name, boost::python::extract<VarMap>( val ) );
        }
        else if ( type == "Vec2" )
        {
            m_node->setVar<Vec2>( name, boost::python::extract<Vec2>( val ) );
        }
        else
        {
            m_node->error( "unable to set var %s to unknown python type %s", name.c_str(), type.c_str() );
        }
    }
    
    bool PyNode::hasVar( const std::string& name )
    {
        return m_node->hasVar<bool>(name) || 
               m_node->hasVar<int>(name) || 
               m_node->hasVar<float>(name) || 
               m_node->hasVar<std::string>(name) || 
               m_node->hasVar<VarMap>(name) ||
               m_node->hasVar<Vec2>(name);
    }
    
    void PyNode::addFunc( boost::python::object tret, boost::python::object targ, boost::python::object func )
    {
        std::string name = boost::python::extract<std::string>(func.attr("__name__"));
        m_node->removeFunc(name);
        m_node->m_funcs[name] = PyFuncUtil::bind( tret, targ, func );
    }
    
    bool PyNode::equals( PyNode* pynode )
    {
        return pynode && m_node.get() == pynode->m_node.get();
    }
    
    boost::python::object PyNode::buildNode( const std::string& type, const std::string& name )
    {
        return create( NodeFactory::instance().buildNode( type, name ) );
    }
    
    void PyNode::log( const std::string& s )
    {
        m_node->log(s.c_str());
    }
    
    void PyNode::publish( const std::string& cmd, boost::python::object obj )
    {
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        
        if ( type == "bool" )
        {
            m_node->publish<bool>(cmd,boost::python::extract<bool>(obj));
        }
        else if ( type == "int" )
        {
            m_node->publish<int>(cmd,boost::python::extract<int>(obj));
        }
        else if ( type == "float" )
        {
            m_node->publish<float>(cmd,boost::python::extract<float>(obj));
        }
        else if ( type == "str" )
        {
            m_node->publish<std::string>(cmd,boost::python::extract<std::string>(obj));
        }
        else if ( type == "VarMap" )
        {
            m_node->publish<VarMap>(cmd,boost::python::extract<VarMap>(obj));
        }
        else if ( type == "Vec2" )
        {
            m_node->publish<Vec2>(cmd,boost::python::extract<Vec2>(obj));
        }
        else
        {
            throw std::runtime_error( "Unable to bind type " + type + " to publish " + cmd );
        }
    }
}
