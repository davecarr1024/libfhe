#ifndef VAR_H
#define VAR_H

#include <typeinfo>
#include <cassert>
#include "PyConverter.h"

namespace gge
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
                    virtual boost::python::object toPy() const =0;
                    
                    template <class T>
                    bool is()
                    {
                        return typeid(T) == getType();
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
                    T m_data;
                    
                public:
                    Data( const T& data ) :
                        m_data(data)
                    {
                    }
                    
                    const std::type_info& getType()
                    {
                        return typeid(T);
                    }
                    
                    AbstractData* clone()
                    {
                        return new Data<T>(m_data);
                    }
                    
                    void set( const T& data )
                    {
                        m_data = data;
                    }
                    
                    T get()
                    {
                        return m_data;
                    }
                    
                    boost::python::object toPy() const
                    {
                        return PyConverter::instance().toPy<T>(m_data);
                    }
            };
            
            AbstractData* m_data;
            
        public:
            Var();
            Var( const Var& var );
            Var& operator=( const Var& var );
            ~Var();
                        
            bool empty();
            void clear();
            
            template <class T>
            bool is() const
            {
                return m_data && m_data->is<T>();
            }
            
            template <class T>
            T get() const
            {
                assert(is<T>());
                return m_data->cast<T>()->get();
            }
            
            template <class T>
            T get( const T& def ) const
            {
                return is<T>() ? m_data->cast<T>()->get() : def;
            }
            
            template <class T>
            void set( const T& val )
            {
                if ( is<T>() )
                {
                    m_data->cast<T>()->set(val);
                }
                else
                {
                    clear();
                    m_data = new Data<T>(val);
                }
            }
            
            boost::python::object toPy() const;
            
            static Var fromPy( boost::python::object obj );
    };
    
}

#endif
