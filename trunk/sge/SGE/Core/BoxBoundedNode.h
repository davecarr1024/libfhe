#ifndef BOX_BOUNDED_NODE
#define BOX_BOUNDED_NODE

#include "SpatialNode3.h"
#include "SGE/Math/AlignedBox.h"

namespace SGE
{
    
    class BoxBoundedNode : public SpatialNode3
    {
        private:
            AlignedBox m_box;
            AlignedBox m_setBox;
            bool m_boxWasSet;
            bool m_boxValid;
            
            void updateBox();
        
        public:
            BoxBoundedNode();
            virtual ~BoxBoundedNode();
            
            void setBox(const AlignedBox& box);
            
            virtual void onMoved();
            
            virtual void getContent(Vec3List& vl) {}
            
            bool overlaps(NodePtr node);
    };
    
    DECLARE_NODE_PTR(BoxBoundedNode);
    
}

#endif
