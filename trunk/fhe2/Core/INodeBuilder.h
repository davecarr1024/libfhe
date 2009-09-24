#ifndef INODEBUILDER_H
#define INODEBUILDER_H

namespace fhe
{
    class Node;
    
    class INodeBuilder
    {
        friend class Node;
        
        public:
            virtual Node* buildNode()=0;
    };
}

#endif
