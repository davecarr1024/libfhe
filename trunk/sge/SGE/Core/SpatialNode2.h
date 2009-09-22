#include "Node.h"
#include "SGE/Math/Vec2.h"
#include "SGE/Math/Rot.h"
#include "SGE/Math/Mat3.h"

namespace SGE
{
    class SpatialNode2 : public Node
    {
        private:
            Vec2 m_position;
            Rot m_rotation;
            Mat3 m_mat, m_invMat, m_globalMat, m_invGlobalMat;
            bool m_matValid, m_globalMatValid;
            
            void updateMat();
            void updateGlobalMat();
            bool allAncestorsGlobalMatValid() const;
            
        public:
            SpatialNode2();
            virtual ~SpatialNode2();
            
            Vec2 getPosition() const;
            Rot getRotation() const;
            
            Mat3 getMat();
            Mat3 getInvMat();
            
            Mat3 getGlobalMat();
            Mat3 getInvGlobalMat();
            
            Mat3 getRelativeMat( NodePtr node );
            
            void setPosition(const Vec2& position);
            void setRotation(const Rot& rotation);
            
            virtual void onAttach();
            virtual void onDetach();
    };

    DECLARE_NODE_PTR(SpatialNode2);
    
}
