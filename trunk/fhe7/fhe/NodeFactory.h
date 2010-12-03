#ifndef FHE_NODE_FACTORY_H
#define FHE_NODE_FACTORY_H

#include <fhe/Node.h>

namespace fhe
{
    
    class INodeRegisterer;
    class FuncRegisterer;
    class DepRegisterer;
    class INodeIntRegisterer;
    
    class NodeFactory
    {
        private:
            friend class INodeRegisterer;
            friend class FuncRegisterer;
            friend class VarRegisterer;
            friend class DepRegisterer;
            friend class INodeIntRegisterer;
            
            std::map< std::string, INodeDescPtr > m_nodes;
            std::map< std::string, INodeIntDescPtr > m_nodeInts;
            
            NodeFactory();
            NodeFactory( const NodeFactory& nf );
            void operator=( const NodeFactory& nf );
            
            void addNode( const INodeDescPtr& node );
            
            void addNodeInt( const INodeIntDescPtr& nodeInt );
            
        public:
            virtual ~NodeFactory();
            
            static NodeFactory& instance();
            
            void init( Node* node ) const;
            
            NodePtr build( const std::string& name ) const;
            
            bool hasNode( const std::string& name ) const;

            INodeDescPtr getNode( const std::string& name ) const;
            
            bool hasNodeInt( const std::string& name ) const;
            
            INodeIntDescPtr getNodeInt( const std::string& name ) const;
    };

    class INodeRegisterer
    {
        protected:
            void addNode( const INodeDescPtr& node )
            {
                NodeFactory::instance().addNode( node );
            }
    };

    template <class T>
    class NodeRegisterer  : public INodeRegisterer
    {
        public:
            NodeRegisterer( const std::string& name )
            {
                addNode( INodeDescPtr( new NodeDesc<T>( name ) ) );
            }
    };
    
    #define FHE_NODE( name ) NodeRegisterer<name> g_##name##_node_reg( #name );
    
    class FuncRegisterer
    {
        public:
            #define FUNCREG_id( z, n, data ) data
            
            #define FUNCREG_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                FuncRegisterer( const std::string& nodeName, const std::string& funcName, \
                    TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) ) { \
                    NodeFactory& nf = NodeFactory::instance(); \
                    if ( nf.hasNode( nodeName ) ) { \
                        nf.getNode( nodeName )->addFunc( \
                            IFuncDescPtr( new FuncDesc<TObj, TRet, \
                            BOOST_PP_ENUM_PARAMS( n, TArg ) \
                            BOOST_PP_COMMA_IF( n ) \
                            BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCREG_id, void ) \
                            > ( funcName, func ) ) ); \
                    } else if ( nf.hasNodeInt( nodeName ) ) { \
                        nf.getNodeInt( nodeName )->addFunc( \
                            IFuncDescPtr( new FuncDesc<TObj, TRet, \
                            BOOST_PP_ENUM_PARAMS( n, TArg ) \
                            BOOST_PP_COMMA_IF( n ) \
                            BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCREG_id, void ) \
                            > ( funcName, func ) ) ); \
                    } else { \
                        FHE_ERROR( "unable to attach func %s to unknown node type or int %s", \
                            funcName.c_str(), nodeName.c_str() ); \
                    } \
                }
                
            BOOST_PP_REPEAT( FHE_ARGS, FUNCREG_iter, ~ )

            #undef FUNCREG_id
            #undef FUNCREG_iter
    };
    
    #define FHE_FUNC( node, func ) FuncRegisterer g_##node##_##func##_func_reg( #node, #func, &node::func );
    
    class VarRegisterer
    {
        public:
            template <class TObj, class TVar>
            VarRegisterer( const std::string& nodeName, const std::string& varName, TVar (TObj::*ptr ) )
            {
                NodeFactory& nf = NodeFactory::instance();
                if ( nf.hasNode( nodeName ) )
                {
                    INodeDescPtr node = nf.getNode( nodeName );
                    node->addVar( IVarDescPtr( new VarDesc<TObj,TVar>( varName, ptr ) ) );
                }
                else if ( nf.hasNodeInt( nodeName ) )
                {
                    INodeIntDescPtr nodeInt = nf.getNodeInt( nodeName );
                    nodeInt->addVar( IVarDescPtr( new VarDesc<TObj,TVar>( varName, ptr ) ) );
                }
                else
                {
                    FHE_ERROR( "unable to attach var %s to unknown node type or int %s",
                               varName.c_str(), nodeName.c_str() );
                }
            }
    };
    
    #define FHE_VAR( node, var ) VarRegisterer g_##node##_##var##_var_reg( #node, #var, &node::var );
    
    class DepRegisterer
    {
        public:
            DepRegisterer( const std::string& nodeName, const std::string& depName )
            {
                NodeFactory& nf = NodeFactory::instance();
                INodeDescPtr node = nf.getNode( nodeName );
                FHE_ASSERT_MSG( node, "unable to add dep %s to unknown node %s", depName.c_str(), nodeName.c_str() );
                if ( nf.hasNode( depName ) )
                {
                    node->addDep( nf.getNode( depName ) );
                }
                else if ( nf.hasNodeInt( depName ) )
                {
                    node->addDep( nf.getNodeInt( depName ) );
                }
                else
                {
                    FHE_ERROR( "unable to add unknown dep %s to node %s", depName.c_str(), nodeName.c_str() );
                }
            }
    };
    
    #define FHE_DEP( node, dep ) DepRegisterer g_##node##_##dep##_dep_reg( #node, #dep );

    class INodeIntRegisterer
    {
        protected:
            void addNodeInt( const INodeIntDescPtr& nodeInt )
            {
                NodeFactory::instance().addNodeInt( nodeInt );
            }
    };
    
    template <class T>
    class NodeIntRegisterer : public INodeIntRegisterer
    {
        public:
            NodeIntRegisterer( const std::string& name )
            {
                addNodeInt( INodeIntDescPtr( new NodeIntDesc<T>( name ) ) );
            }
    };
    
    #define FHE_INODE( node ) NodeIntRegisterer<node> g_##node##_int_reg( #node );
    
}

#endif
