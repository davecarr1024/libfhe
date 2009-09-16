#ifndef INODEBUILDER_H
#define INODEBUILDER_H

#include <string>

namespace fhe
{
    
    class Node;
    
    class INodeBuilder
    {
        public:
            virtual Node* buildNode( const std::string& name )=0;
    };
    
}

#endif
