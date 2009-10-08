#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"
#include "AspectFactory.h"
#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>

namespace fhe
{
    class Entity;
    
    typedef Poco::AutoPtr<Entity> EntityPtr;
    typedef std::map<std::string,EntityPtr> EntityMap;
    
    class Entity : public VarMap, public Poco::RefCountedObject
    {
        private:
            Entity( const Entity& entity ) {}
            Entity& operator=( const Entity& entity ) {}
            
            Entity* m_parent;
            EntityMap m_children;
            
            std::string m_name, m_path;
            
            AspectMap m_aspects;
            
            void updatePath();
            
            static std::map<std::string,int> m_nameCounts;
            
        public:
            static EntityPtr root;
            
            Entity( const std::string& name );
            ~Entity();
            
            std::string getName();
            std::string getPath();
            
            void attachToParent( EntityPtr parent );
            void detachFromParent();
            void addChild( EntityPtr child );
            void removeChild( EntityPtr child );
            
            bool hasChild( const std::string& name );
            EntityPtr getChild( const std::string& name );
            EntityPtr getParent();
            
            AspectPtr addAspect( const std::string& name );
            void removeAspect( const std::string& name );
            bool hasAspect( const std::string& name );
            AspectPtr getAspect( const std::string& name );
            
            Var onGetVar( const std::string& name );
            void onSetVar( const std::string& name, const Var& val );
            
            void publish( const std::string& cmd, const VarMap& args );
            
            void pyPublish( const std::string& cmd, boost::python::dict args );
            
            boost::python::object getAttr( const std::string& name );
            
            void setAttr( const std::string& name, boost::python::object obj );
            
            static boost::python::object defineClass();
            
            template <class TRet, class TArg>
            bool hasAspectFunc( const std::string& name )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,TArg>(name) )
                    {
                        return true;
                    }
                }
                return false;
            }
            
            template <class TRet, class TArg>
            TRet aspectCall( const std::string& name, const TArg& arg )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,TArg>(name) )
                    {
                        return i->second->call<TRet,TArg>(name,arg);
                    }
                }
                throw std::runtime_error("no aspects have func " + name);
            }

            template <class TRet>
            TRet aspectCall( const std::string& name )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,void>(name) )
                    {
                        return i->second->call<TRet>(name);
                    }
                }
                throw std::runtime_error("no aspects have func " + name);
            }
    };
    
}

#endif
