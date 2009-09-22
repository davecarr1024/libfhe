#ifndef NODEBUILDER_H
#define NODEBUILDER_H

#include "INodeBuilder.h"
#include "NodeFactory.h"

#include <vector>
#include <string>
#include <map>

namespace SGE
{
    template <class T>
    class NodeBuilder : public INodeBuilder
    {
        private:
            std::string m_type;
        
        public:
            NodeBuilder(const std::string& type) :
                m_type(type)
            {
                NodeFactory::instance().registerBuilder(m_type,this);
            }
            
            virtual ~NodeBuilder() {
                printf("~NodeBuilder %s\n",m_type.c_str());
            }
            
            virtual T* buildNode()
            {
                return new T;
            }
            
            std::string getType()
            {
                return m_type;
            }
    };

#define REGISTER_NODE_TYPE(nodeClass) SGE::NodeBuilder< nodeClass >* g_##nodeClass##Builder = \
                                      new SGE::NodeBuilder< nodeClass >( #nodeClass );

}

#endif
