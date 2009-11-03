#include "Body.h"
#include "BulletUtil.h"
#include "BodyMotionState.h"

namespace gge
{
    
    GGE_ASPECT(Body);
    
    Body::Body()
    {
        addFunc("on_attach",&Body::on_attach,this);
        addFunc("on_detach",&Body::on_detach,this);
    }
    
    void Body::on_attach()
    {
        EntityPtr worldEnt = getEntity()->getApp()->getEntity("World");
        assert(worldEnt);
        btDiscreteDynamicsWorld* dynamicsWorld = worldEnt->getVar<btDiscreteDynamicsWorld*>("dynamicsWorld",0);
        assert(dynamicsWorld);
        
        btCollisionShape* collisionShape = makeShape();
        assert(collisionShape);
        
        Quat rot = getEntity()->getVar<Quat>("rot",Quat());
        Vec3 pos = getEntity()->getVar<Vec3>("pos",Vec3());
        float mass = getEntity()->getVar<float>("mass",1);
        btVector3 inertia;
        
        BodyMotionState* motionState = new BodyMotionState(getEntity(),
            btTransform(BulletUtil::fromQuat(rot),BulletUtil::fromVec(pos)));
        collisionShape->calculateLocalInertia(mass,inertia);
        btRigidBody::btRigidBodyConstructionInfo rbci(mass,motionState,collisionShape,inertia);
        btRigidBody* rigidBody = new btRigidBody(rbci);
        dynamicsWorld->addRigidBody(rigidBody);
        getEntity()->setVar<btRigidBody*>("rigidBody",rigidBody);
    }
    
    void Body::on_detach()
    {
        btRigidBody* rigidBody = getEntity()->getVar<btRigidBody*>("rigidBody",0);
        if ( rigidBody )
        {
            EntityPtr worldEnt = getEntity()->getApp()->getEntity("World");
            assert(worldEnt);
            btDiscreteDynamicsWorld* dynamicsWorld = worldEnt->getVar<btDiscreteDynamicsWorld*>("dynamicsWorld",0);
            assert(dynamicsWorld);
            
            dynamicsWorld->removeRigidBody(rigidBody);
            getEntity()->setVar<btRigidBody*>("rigidBody",0);
        }
    }
    
    btCollisionShape* Body::makeShape()
    {
        return 0;
    }
    
}
