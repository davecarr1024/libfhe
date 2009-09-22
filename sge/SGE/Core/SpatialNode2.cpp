#include "SpatialNode2.h"

namespace SGE
{

    REGISTER_NODE_TYPE(SpatialNode2);

    SpatialNode2::SpatialNode2() :
        Node(),
        m_matValid(false),
        m_globalMatValid(false)
    {
    }
    
    SpatialNode2::~SpatialNode2()
    {
    }
    
    Vec2 SpatialNode2::getPosition() const
    {
        return m_position;
    }
    
    Rot SpatialNode2::getRotation() const
    {
        return m_rotation;
    }
    
    Mat3 SpatialNode2::getMat()
    {
        updateMat();
        return m_mat;
    }
    
    Mat3 SpatialNode2::getInvMat()
    {
        updateMat();
        return m_invMat;
    }
    
    void SpatialNode2::setPosition(const Vec2& position)
    {
        if (!m_position.equals(position))
        {
            m_position = position;
            m_matValid = m_globalMatValid = false;
        }
    }
    
    void SpatialNode2::setRotation(const Rot& rotation)
    {
        if (!m_rotation.equals(rotation))
        {
            m_rotation = rotation;
            m_matValid = m_globalMatValid = false;
        }
    }
    
    void SpatialNode2::updateMat()
    {
        if (!m_matValid)
        {
            m_mat = Mat3::rotation(m_rotation) * Mat3::translation(m_position);
            m_invMat = m_mat.inverse();
            m_matValid = true;
        }
    }
    
    bool SpatialNode2::allAncestorsGlobalMatValid() const
    {
        SpatialNode2Ptr parent = getFirstAncestorOfType<SpatialNode2>();
        if (parent)
            return m_globalMatValid && parent->allAncestorsGlobalMatValid();
        else
            return m_globalMatValid;
    }
    
    Mat3 SpatialNode2::getGlobalMat()
    {
        updateGlobalMat();
        return m_globalMat;
    }
    
    Mat3 SpatialNode2::getInvGlobalMat()
    {
        updateGlobalMat();
        return m_invGlobalMat;
    }
    
    void SpatialNode2::updateGlobalMat()
    {
        if (!allAncestorsGlobalMatValid())
        {
            SpatialNode2Ptr parent = getFirstAncestorOfType<SpatialNode2>();
            if (parent)
                m_globalMat = parent->getGlobalMat() * getMat();
            else
                m_globalMat = getMat();
            m_invGlobalMat = m_globalMat.inverse();
            m_globalMatValid = true;
        }
    }
    
    void SpatialNode2::onAttach()
    {
        m_globalMatValid = false;
    }
    
    void SpatialNode2::onDetach()
    {
        m_globalMatValid = false;
    }
    
    Mat3 SpatialNode2::getRelativeMat(NodePtr node)
    {
        SpatialNode2Ptr snode = node->getFirstAncestorOfType<SpatialNode2>(true);
        if (snode)
            return getInvGlobalMat() * snode->getGlobalMat();
        else
            return getInvGlobalMat();
    }
}
