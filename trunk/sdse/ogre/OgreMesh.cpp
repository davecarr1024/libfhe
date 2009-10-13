#include "OgreMesh.h"
#include "core/Entity.h"

using namespace sdse;

OgreMesh::OgreMesh(std::string _type, std::string _name) : 
    OgreSceneNode(_type,_name),
    ogreEntity(0) {
}

void OgreMesh::onAttach(Entity* ent) {
    OgreSceneNode::onAttach(ent);
    
    updateOgreMeshName(entity->defaultVar("meshName",std::string("")));
}

void OgreMesh::onSetVar(std::string varName, Var val) {
    OgreSceneNode::onSetVar(varName,val);
    
    if (varName == "meshName" && boost::any_cast<std::string>(&val))
        updateOgreMeshName(boost::any_cast<std::string>(val));
}

void OgreMesh::updateOgreMeshName(std::string meshName) {
    if (ogreEntity) {
        if (sceneNode)
            sceneNode->detachObject(ogreEntity);
        if (sceneManager)
            sceneManager->destroyEntity(ogreEntity);
        else
            delete ogreEntity; //?
        ogreEntity = 0;
    }
    
    if (sceneManager && meshName != "") {
        try {
            ogreEntity = sceneManager->createEntity(name,meshName);
        } catch (Ogre::Exception e) {
            printf("ERROR: couldn't open mesh %s: %s\n",meshName.c_str(),e.getFullDescription().c_str());
            ogreEntity = 0;
        }
    }
    
    if (ogreEntity && sceneNode) {
        sceneNode->attachObject(ogreEntity);
    }
}
