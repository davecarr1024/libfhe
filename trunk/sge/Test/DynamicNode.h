#include "SGE/Core/Node.h"

#include <Poco/ClassLibrary.h>

namespace SGE
{
    class DynamicNode : public Node
    {
        public:
            DynamicNode();
            virtual ~DynamicNode();
    };
}

using namespace SGE;
POCO_BEGIN_MANIFEST(Node)
POCO_EXPORT_CLASS(DynamicNode)
POCO_END_MANIFEST
