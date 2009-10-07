#ifndef PYNODE_H
#define PYNODE_H

#include "Node.h"
#include "VarMap.h"

namespace fhe
{
    
    class PyNode
    {
        friend class Node;
        
        private:
            NodePtr m_node;
            
            PyNode( NodePtr node );
            
        public:
            
            class FuncClosure
            {
                private:
                    NodePtr m_node;
                    
                    boost::python::object m_tret, m_targ;
                    
                public:
                    FuncClosure( NodePtr node, boost::python::object tret, boost::python::object targ );
                    
                    void func( boost::python::object func );
                    
                    static boost::python::object defineClass();
            };
            
            ~PyNode();
            
            static boost::python::object create( NodePtr node );
            
            static boost::python::object defineClass();
            
            void release();
            
            std::string getName();
            std::string getType();
            std::string getPath();
            
            boost::python::object getParent();
            bool hasChild( const std::string& name );
            boost::python::object getChild( const std::string& name );
            boost::python::object getRoot();
            boost::python::object getNode( const std::string& path );
            boost::python::object getChildNames();
            
            void attachToParent( PyNode* parent );
            void detachFromParent();
            void addChild( PyNode* child );
            void removeChild( PyNode* child );
            
            boost::python::object callFunc( const std::string& name, boost::python::object arg );
            
            boost::python::object getVar( const std::string& name );
            
            boost::python::object getVarDef( const std::string& name, boost::python::object def );
            
            void setVar( const std::string& name, boost::python::object val );
            
            bool hasVar( const std::string& name );
            
            bool hasFunc( const std::string& name );
            
            bool equals( PyNode* pynode );
            
            boost::python::object buildNode( const std::string& type, const std::string& name );
            
            void log( const std::string& s );
            
            void publish( const std::string& cmd, boost::python::object obj );
            
            boost::python::object func( boost::python::object tret, boost::python::object targ );
            
            static std::string getPyType( boost::python::object obj );
    };
    
}

#endif
