#ifndef INODEBUILDER_H
#define INODEBUILDER_H

#include <string>
#include <Poco/SharedPtr.h>

namespace SGE
{
    class Node;
    
    class INodeBuilder;
    
    typedef Poco::SharedPtr<INodeBuilder> NodeBuilderPtr;
    
    class INodeBuilder
    {
        public:
            virtual Node* buildNode()=0;
    };
};
    
#endif
