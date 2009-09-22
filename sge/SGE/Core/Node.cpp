#include "Node.h"

#include "FileServer.h"

#include <Poco/DOM/NodeList.h>
#include <Poco/DOM/Document.h>
#include <Poco/DOM/DOMParser.h>

#include <sstream>
#include <exception>
#include <algorithm>
#include <cstdarg>
#include <cstdio>

namespace SGE
{

    REGISTER_NODE_TYPE(Node);

    NodeMap Node::m_nodes;
    
    Node::Node() :
        RefCountedObject(),
        m_parent(0),
        m_name("uninitialized"),
        m_path("uninitialized")
    {
    }
    
    void Node::initialize(const std::string& name)
    {
        m_name = Node::makeName(name);
        Node::m_nodes[m_name] = this;
        updatePath();
    }
    
    Node::~Node()
    {
//         printf("~Node %s\n",m_name.c_str());
        detachFromParent();
        clearChildren();
        Node::m_nodes.erase(m_name);
//         printf("/~Node %s\n",m_name.c_str());
    }
    
    void Node::attachToParent(NodePtr parent)
    {
        if (parent != m_parent)
        {
            detachFromParent();
            m_parent = parent;
            if (m_parent)
            {
                m_parent->addChild(NodePtr(this,true));
                updatePath();
                onAttach();
            }
        }
    }
    
    void Node::detachFromParent()
    {
        if (m_parent)
        {
            onDetach();
            Node* parent = m_parent;
            m_parent = 0;
            parent->removeChild(m_name);
            updatePath();
        }
    }
    
    void Node::addChild(NodePtr child)
    {
        if (child && !hasChild(child->m_name))
        {
            m_children.push_back(child);
            child->attachToParent(NodePtr(this,true));
        }
    }
    
    void Node::removeChild(const std::string& childName)
    {
        NodeList::iterator i = std::find_if(m_children.begin(), m_children.end(), NodeNameEquals(childName));
        if (i != m_children.end())
        {
            m_children.erase(i);
            (*i)->detachFromParent();
        }
    }
    
    void Node::clearChildren()
    {
        NodeList children;
        std::copy(m_children.begin(), m_children.end(), std::back_insert_iterator<NodeList>(children));
        m_children.clear();
        for (NodeList::iterator i = children.begin(); i != children.end(); ++i)
            (*i)->detachFromParent();
    }
    
    bool Node::hasChild(const std::string& childName) const
    {
        return std::find_if(m_children.begin(), m_children.end(), NodeNameEquals(childName)) != m_children.end(); 
    }
    
    NodePtr Node::getChild(const std::string& childName) const
    {
        NodeList::iterator i = std::find_if(const_cast<Node*>(this)->m_children.begin(), 
                                             const_cast<Node*>(this)->m_children.end(), 
                                             NodeNameEquals(childName));
        if (i != m_children.end())
            return *i;
        else
            return 0;
    }
    
    NodePtr Node::getParent() const
    {
        return NodePtr(m_parent,true);
    }
    
    NodePtr Node::getRoot() const
    {
        NodePtr root(const_cast<Node*>(this),true);
        for (; root->m_parent; root = root->getParent());
        return root;
    }
    
    NodePtr Node::getObject(const std::string& path) const
    {
        if (path == "/")
            return getRoot();
        else if (path.substr(0,1) == "/")
            return getRoot()->getObject(path.substr(1));
        else
        {
            size_t slashPos = path.find("/");
            if (slashPos != std::string::npos)
            {
                std::string start = path.substr(0,slashPos),
                            end = path.substr(slashPos+1);
                if (start == ".")
                    return getObject(end);
                else if (start == "..")
                {
                    if (!m_parent) throw std::runtime_error("Path underflow");
                    return m_parent->getObject(end);
                }
                else
                {
                    if (!hasChild(start)) throw new std::runtime_error(std::string("Error: no child ") + start);
                    return getChild(start)->getObject(end);
                }
            }
            else
            {
                if (path == ".")
                    return NodePtr(const_cast<Node*>(this),true);
                else if (path == "..")
                {
                    if (!m_parent) throw std::runtime_error("Path underflow");
                    return NodePtr(m_parent,true);
                }
                else
                {
                    if (!hasChild(path)) throw std::runtime_error(std::string("Error: no child ") + path);
                    return getChild(path);
                }
            }
        }
    }
    
    std::string Node::getName() const
    {
        return m_name;
    }
    
    std::string Node::getPath() const
    {
        return m_path;
    }
    
    std::string Node::makeName(const std::string& name)
    {
        if (Node::m_nodes.find(name) == Node::m_nodes.end())
        {
            return name;
        }
        else
        {
            int i = 2;
            std::ostringstream outs;
            std::string newName;
            do
            {
                outs.str("");
                outs << name << "_" << i++;
                newName = outs.str();
            }
            while (Node::m_nodes.find(newName) != Node::m_nodes.end());
            return newName;
        }
    }
    
    void Node::updatePath()
    {
        if (m_parent && m_parent->m_parent)
        {
            m_path = m_parent->m_path + "/" + m_name;
        }
        else if (m_parent)
        {
            m_path = "/" + m_name;
        }
        else
        {
            m_path = "/";
        }
    }
    
    void Node::log(const char* format, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        printf("%s: %s\n",m_path.c_str(),buffer);
    }
    
    void Node::apply( NodePtr node )
    {
        node->visit(this);
        for (NodeList::iterator i = m_children.begin(); i != m_children.end(); ++i)
            (*i)->apply(node);
        node->postVisit(this);
    }
    
    void Node::load( const std::string& filename )
    {
        static Poco::XML::DOMParser parser;
        Poco::XML::Document* document = parser.parse( FileServer::instance().getFile( filename ) );
        if ( document )
        {
            load( document );
        }
    }
    
    void Node::load( Poco::XML::Document* document )
    {
        if ( document )
        {
            Poco::XML::NodeList* nl = document->getElementsByTagName("children");
            if ( nl )
            {
                for (int i = 0; i < nl->length(); ++i)
                {
                    Poco::XML::Element* childElement = dynamic_cast<Poco::XML::Element*>(nl->item(i));
                    if (childElement)
                    {
                        loadChildren( childElement );
                    }
                }
                nl->release();
            }

            nl = document->getElementsByTagName("includes");
            if ( nl )
            {
                for (int i = 0; i < nl->length(); ++i)
                {
                    Poco::XML::Element* childElement = dynamic_cast<Poco::XML::Element*>(nl->item(i));
                    if (childElement)
                    {
                        loadIncludes( childElement );
                    }
                }
                nl->release();
            }

            nl = document->getElementsByTagName("vars");
            if ( nl )
            {
                for (int i = 0; i < nl->length(); ++i)
                {
                    Poco::XML::Element* childElement = dynamic_cast<Poco::XML::Element*>(nl->item(i));
                    if (childElement)
                    {
                        loadVars( childElement );
                    }
                }
                nl->release();
            }
        }
    }
    
    void Node::load( Poco::XML::Element* element )
    {
        if ( element )
        {
            loadChildren( element->getChildElement("children") );
            loadIncludes( element->getChildElement("includes") );
            loadVars( element->getChildElement("vars") );
        }
    }
    
    void Node::loadChildren( Poco::XML::Element* element )
    {
        if ( element )
        {
            Poco::XML::NodeList* nl = element->getElementsByTagName("child");
            if ( nl )
            {
                for (int i = 0; i < nl->length(); ++i)
                {
                    Poco::XML::Element* childElement = dynamic_cast<Poco::XML::Element*>(nl->item(i));
                    if (childElement)
                    {
                        std::string childName;
                        if ( childElement->hasAttribute("name") )
                        {
                            childName = childElement->getAttribute("name");
                        }
                        else
                        {
                            childName = "obj";
                        }
                        
                        std::string childType;
                        if ( childElement->hasAttribute("type") )
                        {
                            childType = childElement->getAttribute("type");
                        }
                        else
                        {
                            childType = "Node";
                        }
                        
                        NodePtr childNode = NodeFactory::instance().buildNode(childType,childName);
                        if (childNode)
                        {
                            addChild(childNode);
                            childNode->load( childElement );
                        }
                        else
                        {
                            log("ERROR: unable to build node type %s", childType.c_str());
                        }
                    }
                }
                nl->release();
            }
        }
    }

    void Node::loadIncludes( Poco::XML::Element* element )
    {
        if ( element )
        {
            Poco::XML::NodeList* nl = element->getElementsByTagName("include");
            if ( nl )
            {
                for (int i = 0; i < nl->length(); ++i)
                {
                    Poco::XML::Element* childElement = dynamic_cast<Poco::XML::Element*>(nl->item(i));
                    if (childElement)
                    {
                        load( childElement->innerText() );
                    }
                }
                nl->release();
            }
        }
    }

    void Node::loadVars( Poco::XML::Element* element )
    {
        log("loadVars");
        if ( element )
        {
            for ( Poco::XML::Node* node = element->firstChild(); node; node = node->nextSibling() )
            {
                log("load var node");
                Poco::XML::Element* child = dynamic_cast<Poco::XML::Element*>(node);
                if ( child && child->hasAttribute( "name" ) )
                {
                    VarPtr childVar = Var::load( child );
                    log("load var element %s %p", child->getAttribute("name").c_str(),childVar.get());
                    if ( childVar )
                    {
                        setVar( child->getAttribute( "name" ), childVar );
                    }
                }
            }
        }
    }

    bool Node::hasVar( const std::string& name )
    {
        return m_vars.find( name ) != m_vars.end();
    }

    void Node::setVar( const std::string& name, VarPtr val )
    {
        m_vars[name] = val;
    }
    
    VarPtr Node::getVar( const std::string& name )
    {
        return getVar( name, 0 );
    }
    
    VarPtr Node::getVar( const std::string& name, VarPtr def )
    {
        VarMap::iterator i = m_vars.find( name );
        if ( i != m_vars.end() )
            return i->second;
        else
            return def;
    }
    
    VarPtr Node::defaultVar( const std::string& name, VarPtr val )
    {
        if ( !hasVar( name ) )
            setVar( name, val );
        return getVar( name );
    }
    
    void Node::removeVar( const std::string& name )
    {
        m_vars.erase( name );
    }
}
