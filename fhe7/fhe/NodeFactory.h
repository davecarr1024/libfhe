#ifndef FHE_NODE_FACTORY_H
#define FHE_NODE_FACTORY_H

#include <fhe/Node.h>

namespace fhe
{
    
    class INodeRegisterer;
    class FuncRegisterer;
    
    class NodeFactory
    {
        private:
            friend class INodeRegisterer;
            friend class FuncRegisterer;
            friend class VarRegisterer;
            
            std::map< std::string, INodeDescPtr > m_nodes;
            
            NodeFactory();
            NodeFactory( const NodeFactory& nf );
            void operator=( const NodeFactory& nf );
            
            void addNode( const INodeDescPtr& node );
            
            INodeDescPtr getNode( const std::string& name ) const;
            
        public:
            virtual ~NodeFactory();
            
            static NodeFactory& instance();
            
            NodePtr build( const std::string& name ) const;
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
                    INodeDescPtr node = NodeFactory::instance().getNode( nodeName ); \
                    FHE_ASSERT_MSG( node, "can't attach func %s to unknown node %s", funcName.c_str(), nodeName.c_str() ); \
                    node->addFunc( IFuncDescPtr( new FuncDesc<TObj, TRet, \
                        BOOST_PP_ENUM_PARAMS( n, TArg ) \
                        BOOST_PP_COMMA_IF( n ) \
                        BOOST_PP_ENUM( BOOST_PP_SUB( FHE_ARGS, n ), FUNCREG_id, void ) \
                        > ( funcName, func ) ) ); \
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
                INodeDescPtr node = NodeFactory::instance().getNode( nodeName );
                FHE_ASSERT_MSG( node, "unable to add var %s to unknown node %s", varName.c_str(), nodeName.c_str() );
                node->addVar( IVarDescPtr( new VarDesc<TObj,TVar>( varName, ptr ) ) );
            }
    };
    
    #define FHE_VAR( node, var ) VarRegisterer g_##node##_##var##_var_reg( #node, #var, &node::var );
    
}

#endif
