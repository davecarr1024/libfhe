#ifndef NODE_H
#define NODE_H

#include "NodeBuilder.h"
#include "VarMap.h"
#include "FuncMap.h"

#include <string>
#include <vector>
#include <map>

#include <boost/intrusive_ptr.hpp>

namespace fhe
{
    
    #define FHE_NODE_DECL(type) \
        typedef boost::intrusive_ptr<type> type##Ptr; \
        typedef std::vector<type##Ptr> type##List; \
        typedef std::map<std::string, type##Ptr> type##Map;
        
    class Node;
    
    FHE_NODE_DECL(Node);
    
    class Node : public VarMap, public FuncMap
    {
        friend class NodeFactory;
        friend class INodeBuilder;
        
        private:
            int m_refCount;
            
            Node* m_parent;
            NodeMap m_children;
            
            std::string m_name, m_type;
            
            static boost::python::object m_mainNamespace, m_mainModule, m_addFunc;
            
            static bool m_pythonInitialized;
            
            static void initializePython();
            
            void init( const std::string& type, const std::string& name );

        public:
            Node();
            
            friend void intrusive_ptr_add_ref(Node* p);
            friend void intrusive_ptr_release(Node* p);
            
            static boost::python::object defineClass();
            
            void attachToParent( NodePtr parent );
            void detachFromParent();
            void addChild( NodePtr child );
            void removeChild( NodePtr child );
            void clearChildren();
            void release();
            
            NodePtr getParent();
            bool hasChild( const std::string& name );
            NodePtr getChild( const std::string& name );
            
            boost::python::object evalScript( const std::string& s);
            boost::python::object tryEvalScript( const std::string& s);
            void runScript( const std::string& filename );

            boost::python::object toPy();
            
            void onSetVar( const std::string& name, const Var& val );
            void onGetVar( const std::string& name );
            
            void publish( const std::string& cmd, const VarMap& args );
            
            boost::python::object pyBuildNode( const std::string& type, const std::string& name );
            bool pyEquals( NodePtr node );
    };
    
}

#endif
