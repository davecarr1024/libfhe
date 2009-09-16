#ifndef IVAR_H
#define IVAR_H

#include <boost/function.hpp>
#include <boost/python.hpp>
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
            
            virtual boost::python::object getPy()=0;
            virtual void setPy( boost::python::object obj )=0;
            virtual bool canSetPy( boost::python::object obj )=0;
            
            static IVarWrapper* newPy( boost::python::object obj );
    };
    
    template <class TVal>
    class IVar : public IVarWrapper
    {
        public:
            virtual void set(const TVal& val)=0;
            virtual TVal get()=0;
            
            virtual bool canSet()=0;
            virtual bool canGet()=0;
            
            boost::python::object getPy()
            {
                assert( canGet() );
                return boost::python::object( get() );
            }
            
            bool canSetPy( boost::python::object obj )
            {
                boost::python::extract<TVal&> e(obj);
                return canSet() && e.check();
            }
            
            void setPy( boost::python::object obj )
            {
                assert( canSet() );
                boost::python::extract<TVal&> e(obj);
                assert( e.check() );
                set( e() );
            }
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
    
    template <class TVal>
    class FuncVar : public IVar<TVal>
    {
        private:
            typedef boost::function<void (const TVal&)> SetFunction;
            typedef boost::function<TVal()> GetFunction;
            
            SetFunction m_set;
            GetFunction m_get;
            
        public:
            FuncVar(SetFunction set, GetFunction get) :
                m_set(set),
                m_get(get)
            {
            }
            
            void set(const TVal& val)
            {
                m_set(val);
            }
            
            TVal get()
            {
                return m_get();
            }
            
            bool canSet()
            {
                return m_set;
            }
            
            bool canGet()
            {
                return m_get;
            }
    };
}
    
#endif
