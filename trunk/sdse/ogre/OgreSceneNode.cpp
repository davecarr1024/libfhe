#include "OgreSceneNode.h"
#include "core/Entity.h"
#include "core/App.h"

using namespace sdse;

OgreGraphicsServer* OgreSceneNode::graphicsServer = 0;

OgreSceneNode::OgreSceneNode(std::string _type, std::string _name) :
    Aspect("OgreSceneNode",_type,_name),
    sceneNode(0),
    parentSceneNode(0),
    sceneManager(0) {
}

OgreGraphicsServer* OgreSceneNode::getGraphicsServer() {
    if (!graphicsServer)
        graphicsServer = new OgreGraphicsServer(entity->getApp());
    return graphicsServer;
}

void OgreSceneNode::onAttach(Entity* ent) {
    OgreGraphicsServer* graphicsServer = getGraphicsServer();
    if (graphicsServer) {
        sceneManager = graphicsServer->getSceneManager();
        if (sceneManager) {
            sceneNode = sceneManager->getRootSceneNode()->createChildSceneNode(name);
            parentSceneNode = sceneManager->getRootSceneNode();
            
            updateParent(entity->defaultVar("parent",std::string()));
            updatePosition(entity->defaultVar("position",Vector3()));
            updateRotation(entity->defaultVar("rotation",Quaternion()));
            updateScale(entity->defaultVar("scale",Vector3(1,1,1)));
            updateShown(entity->defaultVar("shown",true));
            updateShowBoundingBox(entity->defaultVar("showBoundingBox",false));
        }
    }
}

void OgreSceneNode::onDetach(Entity* ent) {
    if (sceneNode && parentSceneNode) {
        parentSceneNode->removeAndDestroyChild(sceneNode->getName());
        sceneNode = 0;
    }
}

void OgreSceneNode::onSetVar(std::string varName, Var val) {
    if (varName == "parent" && boost::any_cast<std::string>(&val))
        updateParent(boost::any_cast<std::string>(val));
    else if (varName == "position" && boost::any_cast<Vector3>(&val))
        updatePosition(boost::any_cast<Vector3>(val));
    else if (varName == "rotation" && boost::any_cast<Quaternion>(&val))
        updateRotation(boost::any_cast<Quaternion>(val));
    else if (varName == "scale" && boost::any_cast<Vector3>(&val))
        updateScale(boost::any_cast<Vector3>(val));
    else if (varName == "shown" && boost::any_cast<bool>(&val))
        updateShown(boost::any_cast<bool>(val));
    else if (varName == "showBoundingBox" && boost::any_cast<bool>(&val))
        updateShowBoundingBox(boost::any_cast<bool>(val));
}

void OgreSceneNode::updateParent(std::string parentName) {
    if (sceneNode) {
        if (parentSceneNode) parentSceneNode->removeChild(sceneNode);
        
        parentSceneNode = 0;
        
        Entity* parentEntity = entity->getApp()->getEntity(parentName);
        if (parentEntity) {
            OgreSceneNode* parentAspect = (OgreSceneNode*)parentEntity->getFirstAspectOfArchetype("OgreSceneNode");
            if (parentAspect)
                parentSceneNode = parentAspect->sceneNode;
        }
        
        if (!parentSceneNode && sceneManager)
            parentSceneNode = sceneManager->getRootSceneNode();
        
        if (parentSceneNode)
            parentSceneNode->addChild(sceneNode);
    }
}

void OgreSceneNode::updatePosition(Vector3 position) {
    if (sceneNode)
        sceneNode->setPosition(OgreGraphicsServer::vectorToOgreVector(position));
}

void OgreSceneNode::updateRotation(Quaternion rotation) {
    if (sceneNode)
        sceneNode->setOrientation(OgreGraphicsServer::quaternionToOgreQuaternion(rotation));
}

void OgreSceneNode::updateScale(Vector3 scale) {
    if (sceneNode)
        sceneNode->setScale(OgreGraphicsServer::vectorToOgreVector(scale));
}

void OgreSceneNode::updateShown(bool shown) {
    if (sceneNode)
        sceneNode->setVisible(shown);
}

void OgreSceneNode::updateShowBoundingBox(bool showBoundingBox) {
    if (sceneNode)
        sceneNode->showBoundingBox(showBoundingBox);
}
