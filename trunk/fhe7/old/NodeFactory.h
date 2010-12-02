#ifndef FHE_NODE_FACTORY_H
#define FHE_NODE_FACTORY_H

#include <fhe/NodeDesc.h>
#include <fhe/FuncDesc.h>

namespace fhe
{
    
    class NodeFactory
    {
        private:
            std::vector< INodeDescPtr > m_nodes;
            
            NodeFactory();
            NodeFactory( const NodeFactory& nf );
            void operator=( const NodeFactory& nf );
            
        public:
            ~NodeFactory();
            
            void addNode( const INodeDescPtr& node );
            
            INodeDescPtr getNode( const std::string& name ) const;
            
            static NodeFactory& instance();
            
            NodePtr build( const std::string& name ) const;
            
            void init( Node* node ) const;
    };
    
    template <class T>
    class NodeRegisterer
    {
        public:
            NodeRegisterer( const std::string& name )
            {
                NodeFactory::instance().addNode( INodeDescPtr( new NodeDesc<T>( name ) ) );
            }
    };
    
    #define FHE_NODE( name ) NodeRegisterer<name> g_##name##_registerer( #name );

    class FuncRegisterer
    {
        public:
            #define FUNCREG_id( z, n, data ) data
            
            #define FUNCREG_iter( z, n, unused ) \
                template <class TObj, class TRet BOOST_PP_COMMA_IF( n ) BOOST_PP_ENUM_PARAMS( n, class TArg )> \
                FuncRegisterer( const std::string& nodeName, const std::string& funcName, \
                    TRet (TObj::*func)( BOOST_PP_ENUM_PARAMS( n, TArg ) ) ) { \
                    INodeDescPtr node = NodeFactory::instance().getNode( nodeName ); \
                    assert( node ); \
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
    
    #define FHE_FUNC( node, func ) FuncRegisterer g_##node##_##func##_registerer( #node, #func, &node::func );
    
}

#endif
