#include <fhe/PyNode.h>
#include <fhe/NodeFactory.h>

namespace fhe
{
    PyNode::PyNode( const NodePtr& node ) :
        m_node( node )
    {
        FHE_ASSERT( m_node );
    }
    
    PyNode::PyNode( const std::string& type ) :
        m_node( NodeFactory::instance().build( type ) )
    {
        FHE_ASSERT( m_node );
    }
    
    PyNode::~PyNode()
    {
    }
    
    boost::python::object PyNode::defineClass()
    {
        boost::python::scope s = boost::python::class_<PyNode>( "Node", boost::python::init<std::string>() )
            .def( "parent", &PyNode::parent, boost::python::return_value_policy< boost::python::manage_new_object >() )
            .def( "root", &PyNode::root, boost::python::return_value_policy< boost::python::manage_new_object >() )
            .def( "hasChild", &PyNode::hasChild )
            .def( "children", &PyNode::children )
            .def( "attachToParent", &PyNode::attachToParent )
            .def( "detachFromParent", &PyNode::detachFromParent )
            .def( "attachChild", &PyNode::attachChild )
            .def( "detachChild", &PyNode::detachChild )
            .def( "__getattr__", &PyNode::getAttr )
            .def( "__setattr__", &PyNode::setAttr )
            .def( "__eq__", &PyNode::eq )
            .add_property( "func_names", &PyNode::funcNames )
            .add_property( "var_names", &PyNode::varNames )
        ;
        
        boost::python::class_<PyNode::Call>( "Call", boost::python::no_init )
            #define CALL_iter( z, n, unused ) \
                .def( "__call__", &PyNode::Call::BOOST_PP_CAT( call, n ) )
            BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
            #undef CALL_iter
        ;
        
        NodeFactory& nf = NodeFactory::instance();
        boost::python::list types;
        for ( NodeFactory::NodeIterator i = nf.nodesBegin(); i != nf.nodesEnd(); ++i )
        {
            types.append( i->first );
        }
        for ( NodeFactory::NodeIntIterator i = nf.nodeIntsBegin(); i != nf.nodeIntsEnd(); ++i )
        {
            types.append( i->first );
        }
        s.attr( "types" ) = types;
        
        return s;
    }
    
    PyNode::Call::Call( const NodePtr& node, const std::string& name ) :
        m_node( node ),
        m_name( name )
    {
    }
    
    #define CALL_arg( z, n, unused ) boost::python::object BOOST_PP_CAT( arg, n )
    
    #define CALL_app( z, n, unused ) args.push_back( Val( BOOST_PP_CAT( arg, n ) ) );
    
    #define CALL_iter( z, n, unused ) \
        boost::python::object PyNode::Call::BOOST_PP_CAT( call, n )( BOOST_PP_ENUM( n, CALL_arg, ~ ) ) { \
            std::vector< Val > args; \
            BOOST_PP_REPEAT( n, CALL_app, ~ ) \
            return m_node->call( m_name, args ).toPy(); \
        }
            
    BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
    
    #undef CALL_iter
    #undef CALL_app
    #undef CALL_arg
    
    PyNode* PyNode::parent() const
    {
        NodePtr p = m_node->parent();
        return p ? new PyNode( p ) : 0;
    }
    
    boost::python::object PyNode::getAttr( const std::string& name ) const
    {
        if ( m_node->hasVar( name ) )
        {
            Val v;
            if ( m_node->tryGetVar( name, v ) )
            {
                return v.toPy();
            }
        }
        else if ( m_node->hasFunc( name ) )
        {
            return boost::python::object( Call( m_node, name ) );
        }
        FHE_ERROR( "unable to get unknown attr %s", name.c_str() );
    }
    
    void PyNode::setAttr( const std::string& name, boost::python::object o )
    {
        m_node->setVar( name, Val( o ) );
        FHE_ASSERT_MSG( m_node->trySetVar( name, Val( o ) ), 
                        "unable to set var %s with python value %s", 
                        name.c_str(), PyEnv::instance().toString( o ).c_str() );
    }
    
    PyNode* PyNode::root() const
    {
        return new PyNode( m_node->root() );
    }
    
    bool PyNode::hasChild( PyNode* node ) const
    {
        FHE_ASSERT( node );
        return m_node->hasChild( node->m_node );
    }
    
    boost::python::list PyNode::children() const
    {
        boost::python::list c;
        for ( Node::ChildrenIterator i = m_node->childrenBegin(); i != m_node->childrenEnd(); ++i )
        {
            c.append( new PyNode( *i ) );
        }
        return c;
    }
    
    void PyNode::attachToParent( PyNode* parent )
    {
        FHE_ASSERT( parent );
        m_node->attachToParent( parent->m_node );
    }
    
    void PyNode::detachFromParent()
    {
        m_node->detachFromParent();
    }
    
    void PyNode::attachChild( PyNode* child )
    {
        FHE_ASSERT( child );
        m_node->attachChild( child->m_node );
    }
    
    void PyNode::detachChild( PyNode* child )
    {
        FHE_ASSERT( child );
        m_node->detachChild( child->m_node );
    }
    
    bool PyNode::eq( PyNode* node )
    {
        return node && m_node == node->m_node;
    }
    
    boost::python::list PyNode::funcNames() const
    {
        boost::python::list names;
        for ( Node::FuncIterator i = m_node->funcsBegin(); i != m_node->funcsEnd(); ++i )
        {
            names.append( i->first );
        }
        return names;
    }

    boost::python::list PyNode::varNames() const
    {
        boost::python::list names;
        for ( Node::VarIterator i = m_node->varsBegin(); i != m_node->varsEnd(); ++i )
        {
            names.append( i->first );
        }
        return names;
    }
}
