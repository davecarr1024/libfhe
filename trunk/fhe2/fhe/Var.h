#ifndef VAR_H
#define VAR_H

#include <boost/python.hpp>
#include <typeinfo>
#include <cassert>

namespace fhe
{
    
    class Var
    {
        private:
            
            template <class T>
            class Data;
            
            class AbstractData
            {
                public:
                    virtual const std::type_info& getType()=0;
                    virtual AbstractData* clone()=0;
                    
                    template <class T>
                    bool is()
                    {
                        return getType() == typeid(T);
                    }
                    
                    template <class T>
                    Data<T>* cast()
                    {
                        return is<T>() ? static_cast<Data<T>*>(this) : 0;
                    }
            };
            
            template <class T>
            class Data : public AbstractData
            {
                private:
                    T m_val;
                    
                public:
                    Data( const T& val ) :
                        m_val(val)
                    {
                    }
                    
                    const std::type_info& getType()
                    {
                        return typeid(T);
                    }
                    
                    T get()
                    {
                        return m_val;
                    }
                    
                    void set( const T& val )
                    {
                        m_val = val;
                    }
                    
                    AbstractData* clone()
                    {
                        return new Data<T>(m_val);
                    }
            };
            
            AbstractData* m_data;
            
        public:
            Var();
            
            Var( const Var& var);
            
            Var( boost::python::object obj );
            
            template <class T> 
            Var( const T& val ) :
                m_data( new Data<T>(val) )
            {
            }
            
            ~Var();
            
            Var& operator=( const Var& var );
            
            void clear();
            
            bool empty();
            
            template <class T>
            bool is()
            {
                return m_data && m_data->is<T>();
            }
            
            template <class T>
            T get()
            {
                assert(is<T>());
                return m_data->cast<T>()->get();
            }
            
            template <class T>
            void set( const T& val )
            {
                if (is<T>())
                {
                    m_data->cast<T>()->set(val);
                }
                else
                {
                    clear();
                    m_data = new Data<T>(val);
                }
            }
            
            boost::python::object pyGet();
            
            void pySet( boost::python::object obj );
            
            boost::python::object toPy();
            
            static Var fromPy( boost::python::object obj );
            
            static boost::python::object defineClass();
    };
    
}

#endif
