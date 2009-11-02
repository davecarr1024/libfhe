#ifndef ENTITY_H
#define ENTITY_H

#include "Aspect.h"
#include "VarMap.h"
#include "AutoPtr.h"

#include "tinyxml.h"

#include <stdexcept>

namespace gge
{
    class Entity;
    typedef AutoPtr<Entity> EntityPtr;
    typedef std::map<std::string,EntityPtr> EntityMap;
    typedef std::vector<EntityPtr> EntityList;
    
    class App;
    
    class Entity : public VarMap, public RefCounted
    {
        private:
            AspectMap m_aspects;
            
            std::string m_name;
            
            App* m_app;
            
            void loadVars( TiXmlHandle h );
            void loadAspects( TiXmlHandle h );
            
            void loadTag( TiXmlHandle h, const std::string& tag );
            
        public:
            Entity( const std::string& name );
            
            std::string getName();
            
            App* getApp();
            
            void addAspect( AspectPtr aspect );
            void removeAspect( AspectPtr aspect );
            bool hasAspect( const std::string& name );
            AspectPtr getAspect( const std::string& name );
            
            AspectPtr buildAspect( const std::string& name );
            
            boost::python::object pyGetAspect( const std::string& name );
            boost::python::object pyBuildAspect( const std::string& name );
            
            void attachToApp( App* app );
            void detachFromApp();
            
            Var onGetVar( const std::string& name ) const;
            void onSetVar( const std::string& name, const Var& val );
            bool onHasVar( const std::string& name ) const;
            
            void loadData( TiXmlHandle h );
            
            boost::python::object toPy();
            
            boost::python::object getAttr( const std::string& name );
            
            void setAttr( const std::string& name, boost::python::object val );
            
            static boost::python::object defineClass();
            
            bool hasFuncName( const std::string& name );
            
            template <class TRet, class TArg>
            bool hasFunc( const std::string& name ) const
            {
                for ( AspectMap::const_iterator i = m_aspects.begin(); i != m_aspects.end(); ++i )
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
                throw std::runtime_error("unable to call " + name );
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
                throw std::runtime_error("unable to call " + name );
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
