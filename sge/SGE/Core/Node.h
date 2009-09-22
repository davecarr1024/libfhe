#ifndef NODE_H
#define NODE_H

#include "NodeFactory.h"
#include "NodeBuilder.h"
#include "Notification.h"
#include "MsgDelegate.h"
#include "Var.h"

#include <string>
#include <vector>
#include <map>
#include <typeinfo>

#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>
#include <Poco/DOM/Element.h>

namespace SGE
{

    class Node;
    
    typedef Poco::AutoPtr<Node> NodePtr;
    typedef std::vector<NodePtr> NodeList;
    typedef std::map<std::string, NodePtr> NodeMap;
    
    class Node : public Poco::RefCountedObject
    {
        private:
            static NodeMap m_nodes;

            MsgDelegateMap m_msgDelegates;
            
            std::string m_name;
            std::string m_path;
            
            Node* m_parent;
            NodeList m_children;
            
            VarMap m_vars;
            
            static std::string makeName(const std::string& name);
            void updatePath();

        public:
            Node();
            virtual ~Node();

            void initialize(const std::string& name);
            
            void attachToParent(NodePtr parent);
            void detachFromParent();

            void addChild(NodePtr child);
            void removeChild(const std::string& childName);
            void clearChildren();
            bool hasChild(const std::string& childName) const;
            NodePtr getChild(const std::string& childName) const;
            
            NodePtr getParent() const;
            NodePtr getRoot() const;
            NodePtr getObject(const std::string& path) const;
            
            template <class T> Poco::AutoPtr<T> getFirstAncestorOfType( bool includeSelf = false ) const
            {
                NodePtr node;
                Poco::AutoPtr<T> tnode;
                for (node = includeSelf ? NodePtr(const_cast<Node*>(this),true) : getParent(); 
                    node && !tnode; node = node->getParent())
                    tnode = node.cast<T>();
                return tnode;
            }
            
            template <class T> void getAncestorsOfType(std::vector< Poco::AutoPtr<T> >& nodes, bool includeSelf = false) const
            {
                NodePtr node;
                Poco::AutoPtr<T> tnode;
                for (node = includeSelf ? NodePtr(const_cast<Node*>(this),true) : getParent();
                    node; node = node->getParent())
                {
                    tnode = node.cast<T>();
                    if (tnode)
                        nodes.push_back(tnode);
                }
            }
            
            template <class T> void getChildrenOfType(std::vector< Poco::AutoPtr<T> >& nodes) const
            {
                Poco::AutoPtr<T> node;
                for (NodeList::const_iterator i = m_children.begin(); i != m_children.end(); ++i)
                {
                    node = i->cast<T>();
                    if (node)
                        nodes.push_back(node);
                }
            }
            
            template <class T> void getDescendantsOfType(std::vector< Poco::AutoPtr<T> >& nodes, bool deep = false) const
            {
                Poco::AutoPtr<T> node;
                for (NodeList::const_iterator i = m_children.begin(); i != m_children.end(); ++i)
                {
                    node = i->cast<T>();
                    if (node)
                        nodes.push_back(node);
                    if (deep || !node)
                        (*i)->getDescendantsOfType<T>(nodes);
                }
            }

            std::string getName() const;
            std::string getPath() const;
            
            template <class T> void subscribe(MsgDelegatePtr delegate)
            {
                m_msgDelegates[typeid(T).name()] = delegate;
            }
            
            template <class T> void unsubscribe()
            {
                if (hasSubscription<T>())
                    m_msgDelegates.erase(typeid(T).name());
            }
            
            template <class T> bool hasSubscription() const
            {
                return const_cast<Node*>(this)->m_msgDelegates.find(typeid(T).name()) != 
                    const_cast<Node*>(this)->m_msgDelegates.end();
            }
            
            template <class T> void localPublish(const T& msg)
            {
                if (hasSubscription<T>())
                    m_msgDelegates[typeid(T).name()]->notify(msg);
                
                for (NodeList::iterator i = m_children.begin(); i != m_children.end(); ++i)
                    (*i)->localPublish(msg);
            }
            
            template <class T> void publish(const T& msg)
            {
                getRoot()->localPublish(msg);
            }
            
            void apply( NodePtr node );
            virtual void visit( NodePtr node ) {}
            virtual void postVisit( NodePtr node ) {}
            
            void log(const char* format, ...);
            
            virtual void onAttach() {}
            virtual void onDetach() {}
            
            virtual void load( const std::string& filename );
            virtual void load( Poco::XML::Document* e );
            virtual void load( Poco::XML::Element* e );
            virtual void loadIncludes( Poco::XML::Element* e );
            virtual void loadChildren( Poco::XML::Element* e );
            virtual void loadVars( Poco::XML::Element* e );
            
            bool hasVar( const std::string& name );
            void setVar( const std::string& name, VarPtr val );
            VarPtr getVar( const std::string& name );
            VarPtr getVar( const std::string& name, VarPtr def );
            VarPtr defaultVar( const std::string& name, VarPtr def );
            void removeVar( const std::string& name );

        private:
            
            class NodeNameEquals
            {
                private:
                    std::string m_name;
                    
                public:
                    NodeNameEquals(const std::string name) : m_name(name) {}
                    
                    bool operator()(NodePtr node)
                    {
                        return node->getName() == m_name;
                    }
            };
    };
    
    #define DECLARE_NODE_PTR(nodeClass) typedef Poco::AutoPtr< nodeClass > nodeClass##Ptr; \
                                        typedef std::vector< nodeClass##Ptr > nodeClass##List; \
                                        typedef std::map< std::string, nodeClass##Ptr> nodeClass##Map;

}

#endif
