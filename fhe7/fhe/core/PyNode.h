#ifndef FHE_PY_NODE_H
#define FHE_PY_NODE_H

#include <fhe/core/PyEnv.h>
#include <fhe/core/Node.h>

namespace fhe
{
    class PyNode
    {
        private:
            NodePtr m_node;
            
            class Call
            {
                private:
                    NodePtr m_node;
                    std::string m_name;
                    
                public:
                    Call( const NodePtr& node, const std::string& name );
                    
                    #define CALL_arg( z, n, unused ) boost::python::object BOOST_PP_CAT( arg, n )
                    
                    #define CALL_iter( z, n, unused ) \
                        boost::python::object BOOST_PP_CAT( call, n )( BOOST_PP_ENUM( n, CALL_arg, ~ ) );
                        
                    BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
                    
                    #undef CALL_arg
                    #undef CALL_iter
            };
            
        public:
            PyNode( const NodePtr& node );
            PyNode( const std::string& type );
            virtual ~PyNode();
            
            static boost::python::object defineClass();
            
            PyNode* build() const;
            
            PyNode* parent() const;
            PyNode* root() const;
            bool hasChild( PyNode* child ) const;
            boost::python::list children() const;
            
            void attachToParent( PyNode* parent );
            void detachFromParent();
            void attachChild( PyNode* child );
            void detachChild( PyNode* child );
            
            boost::python::list funcNames() const;
            boost::python::list varNames() const;
            
            boost::python::object getAttr( const std::string& name ) const;
            void setAttr( const std::string& name, boost::python::object o );
            
            bool eq( PyNode* node );
            
            void func( boost::python::object func );
            
            std::string type() const;
    };
}

#endif
