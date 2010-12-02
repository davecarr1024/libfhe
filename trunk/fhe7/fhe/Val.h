#ifndef FHE_VAL_H
#define FHE_VAL_H

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
            };
            
            IData* m_data;
            
        public:
            Val();
            Val( const Val& v );
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
            
            template <class T>
            static Val build( T t )
            {
                Val var;
                var.set<T>( t );
                return var;
            }
            
            bool empty() const;
            void clear();
            
            template <class T>
            bool is() const
            {
                return m_data && m_data->as<T>();
            }
            
            template <class T>
            T get( T def = T() ) const
            {
                return is<T>() ? m_data->as<T>()->get() : def;
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
    };
    
}

#endif
