#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <fhe/Func.h>
#include <map>

namespace fhe
{
    
    class INodeDesc;
    
    class Node
    {
        private:
            friend class INodeDesc;
            
            std::map< std::string, IFuncPtr > m_funcs;
            std::string m_type;
            
            Node( const Node& n );
            void operator=( const Node& n );
            
            void setType( const std::string& type );
            void addFunc( const IFuncPtr& func );
            
        protected:
            Node();
            
        public:
            virtual ~Node();
            
            Val call( const std::string& name, const std::vector< Val >& args, const Val& def = Val() );
    };
    
    typedef boost::shared_ptr< Node > NodePtr;
    
    class INodeDesc
    {
        private:
            std::string m_name;
            std::vector< IFuncDescPtr > m_funcs;
            
            void addFunc( const IFuncDescPtr& func );
            
        protected:
            INodeDesc( const std::string& name );
            
        public:
            void init( NodePtr& node );
            
            std::string name() const;
            
            virtual NodePtr build() const = 0;
    };
    
    typedef boost::shared_ptr< INodeDesc > INodeDescPtr;
    
    template <class T>
    class NodeDesc : public INodeDesc
    {
        public:
            NodeDesc( const std::string& name ) :
                INodeDesc( name )
            {
            }
            
            NodePtr build() const
            {
                return NodePtr( new T );
            }
    };
    
}

#endif
