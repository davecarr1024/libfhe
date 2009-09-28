#ifndef NODE_H
#define NODE_H

#include "VarMap.h"
#include "FuncMap.h"
#include "NodeBuilder.h"

#include <boost/intrusive_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/python.hpp>

#include "tinyxml.h"

#include <string>
#include <vector>
#include <map>
#include <cassert>

namespace fhe
{
    
    #define NODE_DECL(type) \
        typedef boost::intrusive_ptr<type> type##Ptr; \
        typedef std::vector<type##Ptr> type##List; \
        typedef std::map<std::string, type##Ptr> type##Map;

    class Node;
    class PyNode;

    NODE_DECL(Node);
    
    class Node : public VarMap, public FuncMap
    {
        friend class PyNode;
        
        private:
            int m_refCount;
            
            Node* m_parent;
            NodeMap m_children;
            
            std::string m_name, m_type, m_path;
            
            NodePtr getLocalNode( const std::string& path );
            
            static std::map<std::string, int> m_nameCount;
            
            void saveInto( TiXmlNode* node );
            
            static bool m_pythonInitialized;
            
            static boost::python::object m_mainModule, m_mainNamespace, m_nodeClass;
            
            static void initializePython();
            
            void updatePath();
            
            void load( const std::string& path );
            void load( TiXmlHandle h );
            
            static NodePtr create( const std::string& path );
            static NodePtr create( TiXmlHandle h );

        public:
            Node( const std::string& name, const std::string& type );
            virtual ~Node();

            void release();
            
            void attachToParent( NodePtr parent );
            void detachFromParent();
            
            std::string getName();
            std::string getType();
            std::string getPath();
            
            bool hasChild( const std::string& name );
            bool hasChild( NodePtr child );
            NodePtr getChild( const std::string& name );
            NodePtr getParent();
            NodePtr getRoot();
            NodePtr getNode( const std::string& path );
            std::vector<std::string> getChildNames();
            
            void addChild( TiXmlHandle h );
            void addChild( const std::string& path );
            
            void addChild( NodePtr child );
            void removeChild( NodePtr child );
            void removeAllChildren();
            
            void load_vars( TiXmlElement* elem );
            void load_children( TiXmlElement* elem );
            void load_includes( TiXmlElement* elem );
            void load_scripts( TiXmlElement* elem );
            
            void save( const std::string& filename );
            
            TiXmlElement* save_vars();
            TiXmlElement* save_children();
            TiXmlElement* save_name();
            TiXmlElement* save_type();

            friend void intrusive_ptr_add_ref(Node* p);
            friend void intrusive_ptr_release(Node* p);
            
            void runScript( const std::string& filename );
            boost::python::object evalScript( const std::string& s );
            boost::python::object tryEvalScript( const std::string& s );
            
            void log( const char* fmt, ...);
            void error( const char* fmt, ...);

            template <class T>
            void publish( const std::string& name, const T& arg )
            {
                std::string msgName = "msg_" + name;
                std::string unmsgName = "unmsg_" + name;
                
                if (hasFunc<void,T>(msgName))
                {
                    callFunc<void,T>(msgName,arg);
                }
                
                for ( NodeMap::iterator i = m_children.begin(); i != m_children.end(); ++i )
                {
                    i->second->publish(name,arg);
                }
                
                if (hasFunc<void,T>(unmsgName))
                {
                    callFunc<void,T>(unmsgName,arg);
                }
            }
            
            void onSetVar( const std::string& name, const Var& val );
            
            Var onGetVar( const std::string& name );
    };
}

#endif
