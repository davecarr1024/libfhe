#ifndef ASPECT_DESC_H
#define ASPECT_DESC_H

#include "Aspect.h"
#include "FuncDesc.h"
#include <map>

namespace fhe
{
    
    template <class T>
    class AspectDesc;
    
    class AbstractAspectDesc
    {
        public:
            virtual const std::type_info& getType()=0;
            
            virtual std::string getName()=0;
            
            virtual Aspect* build()=0;
            
            virtual void init( Aspect* aspect )=0;
            
            template <class T>
            bool is()
            {
                return typeid(T) == getType();
            }
            
            template <class T>
            AspectDesc<T>* cast()
            {
                return is<T>() ? static_cast<AspectDesc<T>*>(this) : 0;
            }
    };
    
    template <class T>
    class AspectDesc : public AbstractAspectDesc
    {
        private:
            std::map<std::string,AbstractFuncDesc*> m_funcs;
            std::string m_name, m_parent;
            
        public:
            AspectDesc( const std::string& name, const std::string& parent ) :
                m_name(name),
                m_parent(parent)
            {
            }
            
            const std::type_info& getType()
            {
                return typeid(T);
            }
            
            std::string getName()
            {
                return m_name;
            }
            
            bool hasFunc( const std::string& name )
            {
                return m_funcs.find(name) != m_funcs.end();
            }
            
            AbstractFuncDesc* getFunc( const std::string& name )
            {
                return hasFunc(name) ? m_funcs[name] : 0;
            }
            
            void addFunc( AbstractFuncDesc* func )
            {
                if ( func )
                {
                    assert(func->is<T>());
                    if ( hasFunc(func->getName()) )
                    {
                        delete m_funcs[func->getName()];
                    }
                    m_funcs[func->getName()] = func;
                }
            }
            
            Aspect* build()
            {
                return new T;
            }
            
            void init( Aspect* aspect )
            {
                T* t = dynamic_cast<T*>(aspect);
                assert(t);
                for ( std::map<std::string,AbstractFuncDesc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
                {
                    t->addFunc(i->second->cast<T>()->instantiate(t));
                }
                
                if ( !m_parent.empty() )
                {
                    AbstractAspectDesc* parentDesc = AspectFactory::instance().getDesc(m_parent);
                    assert(parentDesc);
                    parentDesc->init(aspect);
                }
            }
    };
    
}

#endif
