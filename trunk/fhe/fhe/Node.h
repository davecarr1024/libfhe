#ifndef NODE_H
#define NODE_H

#include "Var.h"
#include "VarMap.h"
#include "Func.h"
#include "NodeBuilder.h"

#include <boost/intrusive_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/python.hpp>

#include "tinyxml/tinyxml.h"

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
    
    class Node
    {
        friend class PyNode;
        
        private:
            int m_refCount;
            
            Node* m_parent;
            NodeMap m_children;
            
            std::string m_name, m_type, m_path;
            
            std::map<std::string, IVarWrapper*> m_vars;
            std::map<std::string, IFuncWrapper*> m_funcs;

            NodePtr getLocalNode( const std::string& path );
            
            static std::map<std::string, int> m_nameCount;
            
            void doLoad( TiXmlHandle h );
            
            void saveInto( TiXmlNode* node );
            
            static bool m_pythonInitialized;
            
            static boost::python::object m_mainModule, m_mainNamespace, m_nodeClass;
            
            static void initializePython();
            
            void updatePath();
            
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
            
            void addChild( NodePtr child );
            void removeChild( NodePtr child );
            void removeAllChildren();
            
            static NodePtr load( const std::string& path );
            static NodePtr load( TiXmlHandle h );

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
            
            void log( const char* fmt, ...);
            void error( const char* fmt, ...);

            void removeVar( const std::string& name );
            
            void clearVars();

            void removeFunc( const std::string& name );

            void clearFuncs();
            
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
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
            {
                return m_funcs.find( name ) != m_funcs.end() && m_funcs[name]->cast<TRet,TArg>();
            }
            
            template <class TRet, class TArg>
            void addFunc( const std::string& name, IFunc<TRet,TArg>* func )
            {
                assert(func);
                removeFunc(name);
                m_funcs[name] = func;
            }
            
            template <class TObj, class TRet, class TArg>
            void addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj )
            {
                addFunc( name, new Func<TRet,TArg>(boost::bind(method,obj,_1)) );
            }
            
            template <class TObj, class TRet>
            void addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj )
            {
                addFunc( name, new Func<TRet,void>(boost::bind(method,obj)));
            }
            
            template <class TRet, class TArg>
            TRet callFunc( const std::string& name, TArg arg )
            {
                bool has = hasFunc<TRet,TArg>(name);
                assert( has );
                return m_funcs[name]->cast<TRet,TArg>()->call(arg);
            }
            
            template <class TRet>
            TRet callFunc( const std::string& name )
            {
                bool has = hasFunc<TRet,void>(name);
                assert( has );
                return m_funcs[name]->cast<TRet,void>()->call();
            }

            template <class T>
            bool hasVar( const std::string& name )
            {
                std::string getName = "get_" + name;
                return hasFunc<T,void>(getName) || (m_vars.find(name) != m_vars.end() && m_vars[name]->cast<T>());
            }

            template <class T>
            bool canSetVar( const std::string& name )
            {
                std::string setName = "set_" + name;
                return hasFunc<void,T>(setName) || (hasVar<T>(name) && m_vars[name]->cast<T>()->canSet());
            }

            template <class T>
            bool canGetVar( const std::string& name )
            {
                std::string getName = "get_" + name;
                return hasFunc<T,void>(getName) || (hasVar<T>(name) && m_vars[name]->cast<T>()->canGet());
            }

            template <class T>
            void setVar( const std::string& name, const T& val )
            {
                std::string setName = "set_" + name;
                if ( hasFunc<void,T>(setName) )
                {
                    callFunc<void,T>(setName, val );
                }
                else if ( hasVar<T>( name ) )
                {
                    IVar<T>* var = m_vars[name]->cast<T>();
                    assert( var->canSet() );
                    var->set( val );
                }
                else
                {
                    connectVar( name, new Var<T>( val ) );
                }
            }

            template <class T>
            void connectVar( const std::string& name, IVar<T>* val )
            {
                assert(val);
                removeVar( name );
                m_vars[name] = val;
            }

            template <class T>
            T getVar( const std::string& name )
            {
                std::string getName = "get_" + name;
                if ( hasFunc<T,void>(getName) )
                {
                    return callFunc<T>(getName);
                }
                
                assert( canGetVar<T>(name) );
                return m_vars[name]->cast<T>()->get();
            }

            template <class T>
            T getVar( const std::string& name, const T& def )
            {
                std::string getName = "get_" + name;
                if ( hasFunc<T,void>(getName) )
                {
                    return callFunc<T>(getName);
                }
                
                if ( canGetVar<T>(name) )
                {
                    return m_vars[name]->cast<T>()->get();
                }
                else
                {
                    return def;
                }
            }
    };
}

#endif
