#ifndef IVAR_H
#define IVAR_H

#include <boost/function.hpp>
#include <cassert>

namespace fhe
{
    template <class TVal>
    class IVar;
    
    class IVarWrapper
    {
        public:
            virtual ~IVarWrapper() {}
            
            template <class TVal>
            IVar<TVal>* cast()
            {
                return dynamic_cast<IVar<TVal>*>(this);
            }
    };
    
    template <class TVal>
    class IVar : public IVarWrapper
    {
        public:
            virtual void set(const TVal& val)=0;
            virtual TVal get()=0;
            
            virtual bool canSet()=0;
            virtual bool canGet()=0;
    };

    template <class TVal>
    class Var : public IVar<TVal>
    {
        private:
            TVal m_val;
            
        public:
            Var(const TVal val) :
                m_val(val)
            {
            }
            
            void set(const TVal& val)
            {
                m_val = val;
            }
            
            TVal get()
            {
                return m_val;
            }
            
            bool canSet()
            {
                return true;
            }
            
            bool canGet()
            {
                return true;
            }
    };
}
    
#endif
