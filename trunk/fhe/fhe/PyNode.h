#ifndef PYNODE_H
#define PYNODE_H

#include "Node.h"
#include "PyFunc.h"
#include "VarMap.h"

namespace fhe
{
    
    class PyNode
    {
        friend class Node;
        
        private:
            NodePtr m_node;
            
            static bool m_initialized;
            
            static void initialize();
            
            static boost::python::object m_addFunc;
            
            PyNode( NodePtr node );
            
            template <class TArg>
            boost::python::object callFuncWithArg( const std::string& name, const TArg& arg );
            
            boost::python::object callFuncNoArg( const std::string& name );

        public:
            
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
            
            void addFunc( boost::python::object tret, boost::python::object targ, boost::python::object func );
            
            bool equals( PyNode* pynode );
            
            boost::python::object buildNode( const std::string& type, const std::string& name );
            
            void log( const std::string& s );
            
            void publish( const std::string& cmd, boost::python::object obj );
            
            static std::string getPyType( boost::python::object obj );
    };
    
}

#endif
