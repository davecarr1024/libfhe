#include "BoxBoundedNode.h"

namespace SGE
{
    
    REGISTER_NODE_TYPE(BoxBoundedNode);
    
    BoxBoundedNode::BoxBoundedNode() :
        SpatialNode3(),
        m_boxWasSet(false),
        m_boxValid(false)
    {
    }
    
    BoxBoundedNode::~BoxBoundedNode()
    {
    }
    
    void BoxBoundedNode::setBox(const AlignedBox& box)
    {
        m_setBox = box;
        m_boxWasSet = true;
        m_boxValid = false;
    }

    void BoxBoundedNode::updateBox()
    {
        if (!m_boxValid)
        {
            BoxBoundedNodeList children;
            getDescendantsOfType<BoxBoundedNode>(children);
            
            if (children.empty())
            {
                if (m_boxWasSet)
                    m_box = m_setBox;
                else
                    m_box.min = m_box.max = Vec3::ZERO;
            }
            else
            {
                m_box = AlignedBox(children[0]->m_box,getRelativeMat(children[0]));
                
                for (int i = 1; i < children.size(); ++i)
                    m_box.expand(children[i]->m_box,getRelativeMat(children[i]));
            }
            
            Vec3List content;
            getContent(content);
            for (Vec3List::iterator i = content.begin(); i != content.end(); ++i)
                m_box.expand(*i);
            
            BoxBoundedNodePtr parent = getFirstAncestorOfType<BoxBoundedNode>();
            if (parent)
                parent->updateBox();
            
            m_boxValid = true;
        }
    }
    
    void BoxBoundedNode::onMoved()
    {
        m_boxValid = false;
    }
    
    bool BoxBoundedNode::overlaps(NodePtr node)
    {
        updateBox();
        
        BoxBoundedNodePtr bbn = node.cast<BoxBoundedNode>();
        if (bbn)
            return m_box.overlaps(bbn->m_box,getRelativeMat(bbn));
        else
            return false;
    }

}
