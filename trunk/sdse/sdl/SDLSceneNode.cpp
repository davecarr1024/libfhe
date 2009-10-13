#include "SDLSceneNode.h"
#include "SDLGraphicsServer.h"

#include "core/Entity.h"
#include "core/App.h"
#include "core/math/Vector2.h"
#include "core/math/Vector3.h"
#include "core/math/Quaternion.h"

using namespace sdse;

SDLGraphicsServer* SDLSceneNode::_graphicsServer = 0;

SDLSceneNode::SDLSceneNode(std::string _type, std::string _name) :
    Aspect("SDLSceneNode",_type,_name),
    parent(0) {
}

SDLGraphicsServer* SDLSceneNode::getGraphicsServer() {
    if (!_graphicsServer)
        _graphicsServer = new SDLGraphicsServer(entity->getApp());
    return _graphicsServer;
}

void SDLSceneNode::onAttach(Entity* ent) {
    updateParent(ent->defaultVar("parent",std::string("")));
}

void SDLSceneNode::onDetach(Entity* ent) {
    printf("%s detach\n",name.c_str());
    for (SDLSceneNodeList::iterator i = children.begin(); i != children.end(); ++i)
        (*i)->detach();
    detachFromParent();
    printf("/%s detach\n",name.c_str());
}

void SDLSceneNode::updateParent(std::string parentName) {
    if (name == "RootSceneNode") return;
    
    SDLSceneNode* parentSceneNode = 0;

    Entity* parentEntity = entity->getApp()->getEntity(parentName);
    if (parentEntity)
        parentSceneNode = (SDLSceneNode*)parentEntity->getFirstAspectOfArchetype("SDLSceneNode");
    
    if (!parentSceneNode) {
        SDLGraphicsServer* graphicsServer = getGraphicsServer();
        parentSceneNode = graphicsServer->getRootSceneNode();
    }
    
    attachToParent(parentSceneNode);
}

bool SDLSceneNode::hasAncestor(SDLSceneNode* node) {
    return this == node || (parent && parent->hasAncestor(node));
}

void SDLSceneNode::addChild(SDLSceneNode* child) {
    bool hasChild = false;
    for (SDLSceneNodeList::iterator i = children.begin(); i != children.end() && !hasChild; ++i)
        if (*i == child)
            hasChild = true;
        
    if (!hasChild || !child->hasAncestor(this)) {
        child->detachFromParent();
        children.push_back(child);
        child->attachToParent(this);
    }
}

void SDLSceneNode::removeChild(SDLSceneNode* child) {
    bool hasChild = false;
    for (SDLSceneNodeList::iterator i = children.begin(); i != children.end() && !hasChild; ++i)
        if (*i == child)
            hasChild = true;
        
    if (hasChild) {
        for (SDLSceneNodeList::iterator i = children.begin(); i != children.end(); ++i)
            if (*i == child)
                children.erase(i);
        child->detachFromParent();
    }
}

void SDLSceneNode::attachToParent(SDLSceneNode* _parent) {
    if (_parent != parent && !hasAncestor(_parent)) {
        detachFromParent();
        parent = _parent;
        parent->addChild(this);
    }
}

void SDLSceneNode::detachFromParent() {
    if (parent) {
        printf("%s detachFromParent\n",name.c_str());
        SDLSceneNode* _parent = parent;
        parent = 0;
        _parent->removeChild(this);
        printf("/%s detachFromParent\n",name.c_str());
    }
}

void SDLSceneNode::render() {
    bool doMatrix = false, 
         doPosition = false, 
         doRotation = false, 
         doScale = false;
    
    Vector2 position, scale;
    float rotation;
    
    doPosition = entity->queryVar("position",position);
    doRotation = entity->queryVar("rotation",rotation);
    doScale = entity->queryVar("scale",scale);
    doMatrix = doPosition || doRotation || doScale;
    
    if (doMatrix) {
        glPushMatrix();
        if (doPosition)
            glTranslatef(position.x,position.y,0);
        if (doRotation)
            glRotatef(rotation,0,0,1);
        if (doScale)
            glScalef(scale.x,scale.y,1);
    }
    
    bool doAttrib = false,
         doColor = false,
         doAlpha = false;
         
    Vector3 color;
    float alpha;
    doColor = entity->queryVar("color",color);
    doAlpha = entity->queryVar("alpha",alpha);
    doAttrib = doColor || doAlpha;
    
    if (doAttrib) {
        glPushAttrib(GL_CURRENT_BIT);
        
        if (doColor)
            glColor4f(color.x,color.y,color.z,doAlpha ? alpha : 1);
        else if (doAlpha)
            glColor4f(1,1,1,alpha);
    }
    
    geom();
    
    for (SDLSceneNodeList::iterator i = children.begin(); i != children.end(); ++i)
        (*i)->render();
    
    if (doMatrix)
        glPopMatrix();
    if (doAttrib)
        glPopAttrib();
}
