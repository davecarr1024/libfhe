#ifndef ASPECT_DESC_H
#define ASPECT_DESC_H

#include "AbstractAspectDesc.h"
#include "AspectFactory.h"
#include "Aspect.h"

namespace fhe
{
    
    template <class T, class TParent>
    class AspectDesc : public AbstractAspectDesc
    {
        private:
            std::map<std::string,AbstractFuncDesc*> m_funcs;
            std::string m_name;
            
        public:
            AspectDesc( const std::string& name ) :
                m_name(name)
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
                if ( typeid(T) != typeid(TParent) )
                {
                    AbstractAspectDesc* parent = AspectFactory::instance().getDesc<TParent>();
                    assert(parent);
                    parent->init(aspect);
                }
                else
                {
                    assert(typeid(T) == typeid(Aspect));
                }
                
                T* t = dynamic_cast<T*>(aspect);
                assert(t);
                for ( std::map<std::string,AbstractFuncDesc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
                {
                    t->addFunc(i->second->cast<T>()->instantiate(t));
                }
                
            }
    };
    
}

#endif
