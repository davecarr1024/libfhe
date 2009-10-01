#ifndef NODE_H
#define NODE_H

#include "NodeBuilder.h"
#include "VarMap.h"
#include "FuncMap.h"

#include <string>
#include <vector>
#include <map>

#include <boost/intrusive_ptr.hpp>
#include <tinyxml.h>

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
            
            std::string m_name, m_type, m_path;
            
            static boost::python::object m_mainModule;
            static boost::python::dict m_mainNamespace;
            
            static bool m_pythonInitialized;
            
            static void initializePython();
            
            void init( const std::string& type, const std::string& name );
            
            NodePtr loadChildData( TiXmlHandle h );
            
            NodePtr createChild( TiXmlHandle h );
            
            void fillChild( NodePtr child, TiXmlHandle h );
            
        public:
            
            Node();
            virtual ~Node();
            
            friend void intrusive_ptr_add_ref(Node* p);
            friend void intrusive_ptr_release(Node* p);
            
            static boost::python::object defineClass();
            
            void attachToParent( NodePtr parent );
            void pyAttachToParent( boost::python::object parent );
            void detachFromParent();
            void addChild( NodePtr child );
            void pyAddChild( boost::python::object child );
            void removeChild( NodePtr child );
            void pyRemoveChild( boost::python::object child );
            void clearChildren();
            void release();
            
            std::string getName();
            std::string getPath();
            
            NodePtr getParent();
            boost::python::object pyGetParent();
            bool hasChild( const std::string& name );
            NodePtr getChild( const std::string& name );
            boost::python::object pyGetChild( const std::string& name );
            NodePtr getRoot();
            boost::python::object pyGetRoot();
            
            boost::python::dict defaultNamespace();
            void execScript( const std::string& s );
            void execScript( const std::string& s, boost::python::dict ns );
            boost::python::object evalScript( const std::string& s);
            boost::python::object evalScript( const std::string& s, boost::python::dict ns);
            boost::python::object tryEvalScript( const std::string& s);
            boost::python::object tryEvalScript( const std::string& s, boost::python::dict ns);
            void runScript( const std::string& filename, boost::python::dict ns );
            void runScript( const std::string& filename );

            boost::python::object toPy();
            static NodePtr fromPy( boost::python::object obj );
            
            void onSetVar( const std::string& name, const Var& val );
            Var onGetVar( const std::string& name );
            
            void publish( const std::string& cmd, const VarMap& args );
            
            bool pyEquals( boost::python::object node );

            void log( const char* fmt, ...);
            void error( const char* fmt, ...);
            
            NodePtr loadChild( const std::string& filename );
            boost::python::object pyLoadChild( const std::string& filename );
            boost::python::object pyBuildChild( const std::string& type, const std::string& name );
            
            void load_children( TiXmlHandle h );
            void load_vars( TiXmlHandle h );
            void load_scripts( TiXmlHandle h );
            
            void pyLog( const std::string& s );
    };
    
}

#endif
