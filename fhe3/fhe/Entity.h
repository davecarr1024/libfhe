#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"
#include "AspectFactory.h"
#include <Poco/AutoPtr.h>
#include <Poco/RefCountedObject.h>
#include "tinyxml.h"

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
            
            EntityPtr loadChildData( TiXmlHandle h );
            void load( TiXmlHandle h );
            
            void loadTag( TiXmlHandle h, const std::string& tag );
            void loadVars( TiXmlHandle h );
            void loadAspects( TiXmlHandle h );
            void loadChildren( TiXmlHandle h );
            
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
            EntityPtr getRoot();
            
            boost::python::object pyGetChild( const std::string& name );
            boost::python::object pyGetParent();
            boost::python::object pyGetRoot();
            
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
            
            EntityPtr loadChild( const std::string& filename );
            
            EntityPtr buildChild( const std::string& name );
            
            boost::python::object pyLoadChild( const std::string& filename );
            
            boost::python::object pyBuildChild( const std::string& name );
            
            boost::python::object toPy();
            
            static boost::python::object defineClass();
            
            void error( const char* fmt, ...);
            
            bool pyHasFunc( const std::string& name );

            template <class TRet, class TArg>
            bool hasFunc( const std::string& name )
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
            TRet call( const std::string& name, const TArg& arg )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,TArg>(name) )
                    {
                        return i->second->call<TRet,TArg>(name,arg);
                    }
                }
                error("no aspects have func %s", name.c_str());
            }
            
            template <class TRet, class TArg>
            TRet callAll( const std::string& name, const TArg& arg )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,TArg>(name) )
                    {
                        i->second->call<TRet,TArg>(name,arg);
                    }
                }
            }
            
            template <class TRet>
            TRet call( const std::string& name )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,void>(name) )
                    {
                        return i->second->call<TRet>(name);
                    }
                }
                error("no aspects have func %s", name.c_str());
            }

            template <class TRet>
            TRet callAll( const std::string& name )
            {
                for ( AspectMap::iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
                {
                    if ( i->second->hasFunc<TRet,void>(name) )
                    {
                        i->second->call<TRet>(name);
                    }
                }
            }
    };
    
}

#endif
