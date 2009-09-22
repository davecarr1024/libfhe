#include "Node.h"
#include "SGE/Math/Vec3.h"
#include "SGE/Math/Quat.h"
#include "SGE/Math/Mat4.h"

namespace SGE
{
    class SpatialNode3 : public Node
    {
        private:
            Vec3 m_position;
            Quat m_rotation;
            Mat4 m_mat, m_invMat, m_globalMat, m_invGlobalMat;
            bool m_matValid, m_globalMatValid;
            
            void updateMat();
            void updateGlobalMat();
            bool allAncestorsGlobalMatValid() const;
            
        public:
            SpatialNode3();
            virtual ~SpatialNode3();
            
            Vec3 getPosition() const;
            Quat getRotation() const;
            
            Mat4 getMat();
            Mat4 getInvMat();
            
            Mat4 getGlobalMat();
            Mat4 getInvGlobalMat();
            
            void setPosition(const Vec3& position);
            void setRotation(const Quat& rotation);
            
            Mat4 getRelativeMat(NodePtr node);
            
            virtual void onAttach();
            virtual void onDetach();
            virtual void onMoved() {}
    };

    DECLARE_NODE_PTR(SpatialNode3);
    
}
