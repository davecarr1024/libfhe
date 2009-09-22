#include "NodeFactory.h"
#include "Node.h"
#include "FileServer.h"

#include <Poco/SharedLibrary.h>

#include <cassert>

namespace SGE
{
    
    NodeFactory::NodeFactory()
    {
    }
       
    NodeFactory::~NodeFactory()
    {
//         printf("~NodeFactory\n");
    }
    
    NodeFactory& NodeFactory::instance()
    {
        static NodeFactory factory;
        return factory;
    }
    
    void NodeFactory::registerBuilder(const std::string& type, NodeBuilderPtr builder)
    {
        m_builders[type] = builder;
    }
    
    NodePtr NodeFactory::buildNode(const std::string& type, const std::string& name)
    {
        NodePtr node;
        if (m_builders.find(type) != m_builders.end())
            node = m_builders[type]->buildNode();
        else
        {
            std::string libName = type + Poco::SharedLibrary::suffix(),
                        libPath = FileServer::instance().getFile(libName);
                        
            if (!libPath.empty())
            {
                if (!m_loader.isLibraryLoaded(libPath))
                {
                    try
                    {
                        m_loader.loadLibrary(libPath);
                    }
                    catch (const Poco::Exception& e)
                    {
                        printf("NodeFactory: unable to load node lib %s: %s\n",libPath.c_str(),e.displayText().c_str());
                    }
                }
                
                if (m_loader.isLibraryLoaded(libPath))
                {
                    try
                    {
/*                        const Poco::ClassLoader<Node>::Meta* meta = m_loader.findClass(type);
                        if (meta)
                        {
                            Node* n = meta->create();
                            if (n && !meta->isAutoDelete(n))
                            {
                                node = n;
                            }
                        }*/
                        node = m_loader.classFor(type).create();
                    }
                    catch (const Poco::Exception& e)
                    {
                        printf("NodeFactory: unable to create type %s from lib %s: %s\n",
                                type.c_str(),libPath.c_str(),e.displayText().c_str());
                    }
                }
            }
        }
        
        if (node)
            node->initialize(name);
        return node;
    }
}
