#include "SpatialNode3.h"

namespace SGE
{
    
    REGISTER_NODE_TYPE(SpatialNode3);

    SpatialNode3::SpatialNode3() :
        Node(),
        m_matValid(false),
        m_globalMatValid(false)
    {
    }
    
    SpatialNode3::~SpatialNode3()
    {
    }
    
    Vec3 SpatialNode3::getPosition() const
    {
        return m_position;
    }
    
    Quat SpatialNode3::getRotation() const
    {
        return m_rotation;
    }
    
    Mat4 SpatialNode3::getMat()
    {
        updateMat();
        return m_mat;
    }

    Mat4 SpatialNode3::getInvMat()
    {
        updateMat();
        return m_invMat;
    }
    
    void SpatialNode3::setPosition(const Vec3& position)
    {
        if (!m_position.equals(position))
        {
            m_position = position;
            m_globalMatValid = m_matValid = false;
            onMoved();
        }
    }
    
    void SpatialNode3::setRotation(const Quat& rotation)
    {
        if (!m_rotation.equals(rotation))
        {
            m_rotation = rotation;
            m_globalMatValid = m_matValid = false;
            onMoved();
        }
    }
    
    void SpatialNode3::updateMat()
    {
        if (!m_matValid )
        {
            m_mat = Mat4::translation(m_position) * Mat4::rotation(m_rotation);
            m_invMat = m_mat.inverse();
            m_matValid = true;
        }
    }
    
    void SpatialNode3::updateGlobalMat()
    {
        if (!allAncestorsGlobalMatValid())
        {
            SpatialNode3Ptr parent = getFirstAncestorOfType<SpatialNode3>();
            if (parent)
                m_globalMat = parent->getGlobalMat() * getMat();
            else
                m_globalMat = getMat();
            m_invGlobalMat = m_globalMat.inverse();
            m_globalMatValid = true;
        }
    }
    
    bool SpatialNode3::allAncestorsGlobalMatValid() const
    {
        SpatialNode3Ptr parent = getFirstAncestorOfType<SpatialNode3>();
        if (parent)
            return m_globalMatValid && parent->allAncestorsGlobalMatValid();
        else
            return m_globalMatValid;
    }
    
    Mat4 SpatialNode3::getGlobalMat()
    {
        updateGlobalMat();
        return m_globalMat;
    }
    
    Mat4 SpatialNode3::getInvGlobalMat()
    {
        updateGlobalMat();
        return m_invGlobalMat;
    }
    
    void SpatialNode3::onAttach()
    {
        m_globalMatValid = false;
    }
    
    void SpatialNode3::onDetach()
    {
        m_globalMatValid = false;
    }
    
    Mat4 SpatialNode3::getRelativeMat(NodePtr node)
    {
        SpatialNode3Ptr snode = node->getFirstAncestorOfType<SpatialNode3>(true);
        if (snode)
            return getInvGlobalMat() * snode->getGlobalMat();
        else
            return getInvGlobalMat();
    }
    
}
