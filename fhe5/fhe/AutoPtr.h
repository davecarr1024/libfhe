#ifndef AUTOPTR_H
#define AUTOPTR_H

namespace fhe
{
    
    template <class T>
    class AutoPtr
    {
        private:
            T* m_obj;
            
            void inc()
            {
                if ( m_obj )
                {
                    m_obj->incRef();
                }
            }
            
            void dec()
            {
                if ( m_obj && !m_obj->decRef() )
                {
                    delete m_obj;
                    m_obj = 0;
                }
            }
            
        public:
            AutoPtr() : m_obj(0) {}
            
            AutoPtr( T* obj ) : 
                m_obj(obj)
            {
                //printf("autoptr build from pointer %p\n",obj);
                inc();
            }
            
            AutoPtr( const AutoPtr& ptr ) : 
                m_obj(ptr.m_obj)
            {
                //printf("autoptr build from ptr %p\n",ptr.m_obj);
                inc();
            }
            
            AutoPtr& operator=( const AutoPtr& ptr )
            {
                //printf("autoptr assign from %p => %p\n",m_obj,ptr.m_obj);
                dec();
                m_obj = ptr.m_obj;
                inc();
                return *this;
            }
            
            AutoPtr& operator=( T* obj )
            {
                //printf("autoptr assign from %p => %p\n",m_obj,obj);
                dec();
                m_obj = obj;
                inc();
                return *this;
            }
            
            ~AutoPtr()
            {
                //printf("autoptr dtor\n");
                dec();
            }
            
            T* get()
            {
                return m_obj;
            }
            
            template <class U>
            U* cast()
            {
                return dynamic_cast<U*>(m_obj);
            }
            
            operator T*()
            {
                return m_obj;
            }
            
            bool valid()
            {
                return m_obj;
            }
            
            T* operator->() const
            {
                return m_obj;
            }
            
            T& operator*()
            {
                return m_obj;
            }
            
            bool operator!()
            {
                return empty();
            }
            
            bool empty()
            {
                return !m_obj;
            }
            
            bool operator==( const AutoPtr<T>& ptr )
            {
                return m_obj == ptr.m_obj;
            }

            bool operator!=( const AutoPtr<T>& ptr )
            {
                return m_obj != ptr.m_obj;
            }
            
            bool operator<( const AutoPtr<T>& ptr )
            {
                return m_obj < ptr.m_obj;
            }
            
            bool operator<=( const AutoPtr<T>& ptr )
            {
                return m_obj < ptr.m_obj;
            }
            
            bool operator>( const AutoPtr<T>& ptr )
            {
                return m_obj > ptr.m_obj;
            }
            
            bool operator>=( const AutoPtr<T>& ptr )
            {
                return m_obj >= ptr.m_obj;
            }
    };

}

#endif
