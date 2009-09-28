#ifndef SCENENODE_H
#define SCENENODE_H

#include <fhe/Node.h>
#include <fhe/math/Vec3.h>
#include <fhe/math/Quat.h>

#include <Ogre.h>

namespace fhe
{
    class SceneNode : public Node
    {
        private:
            Ogre::SceneNode* m_sceneNode;
            Ogre::MovableObject* m_content;
            
        public:
            SceneNode( const std::string& type, const std::string& name );
            
            void on_attach();
            void on_detach();
            
            virtual Ogre::MovableObject* create( Ogre::SceneManager* sceneManager );
            
            Ogre::SceneManager* getSceneManager();
            Ogre::SceneNode* getSceneNode();
            Ogre::SceneNode* getParentSceneNode();
            
            void set_pos( Var val );
            Var get_pos();
            
            void set_rot( Var val );
            Var get_rot();
            
            void set_scale( Var val );
            Var get_scale();
            
            void setContent( Ogre::MovableObject* content );
    };
    
    NODE_DECL(SceneNode);
}

#endif
