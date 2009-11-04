#ifndef FUNC_H
#define FUNC_H

#include "Var.h"

namespace gge
{
    
    class AbstractFunc
    {
        public:
            virtual Var call( const Var& arg )=0;
    };
    
    template <class TObj>
    class Func : public AbstractFunc
    {
        public:
            typedef Var (TObj::*Method)(const Var&);
            
        private:
            TObj* m_obj;
            Method m_method;
        
        public:
            Func( TObj* obj, Method method ) :
                m_obj(obj),
                m_method(method)
            {
                assert(m_obj);
                assert(m_method);
            }
            
            virtual Var call( const Var& arg ) 
            {
                return (m_obj->*m_method)(arg);
            }
    };
}

#endif
