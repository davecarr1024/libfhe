#ifndef FHE_VAL_H
#define FHE_VAL_H

#include <fhe/Util.h>
#include <fhe/Mat.h>
#include <fhe/Rot.h>
#include <fhe/Vec.h>
#include <boost/python.hpp>
#include <boost/mpl/list.hpp>
#include <string>
#include <typeinfo>

namespace fhe
{
    
    class Val
    {
        private:
            template <class T>
            class Data;
            
            class IData
            {
                public:
                    template <class T>
                    Data<T>* as()
                    {
                        return dynamic_cast<Data<T>*>( this );
                    }
                    
                    virtual IData* clone() const = 0;
                    
                    virtual std::string type() const = 0;
                    
                    virtual boost::python::object toPy() const = 0;
            };
            
            template <class T>
            class Data : public IData
            {
                private:
                    T m_t;
                    
                public:
                    Data( T t ) :
                        m_t( t )
                    {
                    }
                    
                    T get() const
                    {
                        return m_t;
                    }
                    
                    void set( T t )
                    {
                        m_t = t;
                    }
                    
                    IData* clone() const
                    {
                        return new Data<T>( m_t );
                    }
                    
                    std::string type() const
                    {
                        return typeid(T).name();
                    }
                    
                    boost::python::object toPy() const
                    {
                        return boost::python::object( m_t );
                    }
            };
            
            IData* m_data;
            
        public:
            typedef 
                boost::mpl::list< Mat3, Mat2, Rot3, Rot2, Vec3, Vec2, std::string, double, int, bool > 
                python_convertable_types;
            
            Val();
            Val( const Val& v );
            Val( boost::python::object obj );
            Val& operator=( const Val& v );
            ~Val();
            
            template <class T>
            Val( T t ) :
                m_data( new Data<T>( t ) )
            {
            }
            
            template <class T>
            operator T()
            {
                return get<T>();
            }
            
            bool empty() const;
            void clear();
            std::string type() const;
            
            template <class T>
            bool is() const
            {
                return m_data && m_data->as<T>();
            }
            
            template <class T>
            bool tryGet( T& t ) const
            {
                if ( is<T>() )
                {
                    t = m_data->as<T>()->get();
                    return true;
                }
                else
                {
                    return false;
                }
            }
            
            template <class T>
            T get() const
            {
                FHE_ASSERT_MSG( is<T>(), 
                                "type mismatch while doing Val conversion: val is %s but %s requested",
                                type().c_str(), typeid(T).name() );
                return m_data->as<T>()->get();
            }
            
            template <class T>
            void set( T t )
            {
                if ( is<T>() )
                {
                    m_data->as<T>()->set( t );
                }
                else
                {
                    clear();
                    m_data = new Data<T>( t );
                }
            }
            
            boost::python::object toPy() const
            {
                return m_data ? m_data->toPy() : boost::python::object();
            }
    };
    
}

#endif
