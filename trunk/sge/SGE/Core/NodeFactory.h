#ifndef NODEFACTORY_H
#define NODEFACTORY_H

#include "INodeBuilder.h"

#include <string>
#include <map>
#include <Poco/AutoPtr.h>
#include <Poco/ClassLoader.h>

namespace SGE
{
    class Node;
    
    typedef Poco::AutoPtr<Node> NodePtr;
    
    class NodeFactory
    {
        private:
            std::map<std::string, NodeBuilderPtr> m_builders;
            
            Poco::ClassLoader<Node> m_loader;
            
            NodeFactory();

        public:
            virtual ~NodeFactory();
            
            static NodeFactory& instance();
            
            void registerBuilder(const std::string& type, NodeBuilderPtr builder);
            
            NodePtr buildNode(const std::string& type, const std::string& name);
    };
    
}

#endif
