#ifndef AUTOPTR_H
#define AUTOPTR_H

namespace gge
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
                    m_obj->inc();
                }
            }
            
            void dec()
            {
                if ( m_obj && !m_obj->dec() )
                {
                    delete m_obj;
                    m_obj = 0;
                }
            }
            
        public:
            AutoPtr() :
                m_obj(0)
            {
            }
            
            AutoPtr( T* ptr ) :
                m_obj(ptr)
            {
                inc();
            }
            
            AutoPtr( const AutoPtr& ptr ) :
                m_obj(ptr.m_obj)
            {
                inc();
            }
            
            ~AutoPtr()
            {
                dec();
            }
            
            AutoPtr& operator=( const AutoPtr& ptr )
            {
                dec();
                m_obj = ptr.m_obj;
                inc();
            }
            
            AutoPtr& operator=( T* ptr )
            {
                dec();
                m_obj = ptr;
                inc();
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
            
            bool empty()
            {
                return !valid();
            }
            
            T& operator*()
            {
                return m_obj;
            }
            
            T* operator->()
            {
                return m_obj;
            }
            
            bool operator!()
            {
                return empty();
            }
            
            bool operator==( T* ptr )
            {
                return m_obj == ptr;
            }
            
            bool operator!=( T* ptr )
            {
                return m_obj != ptr;
            }
    };
    
    class RefCounted
    {
        private:
            int m_refCount;
            
            RefCounted( const RefCounted& ref )
            {
            }
            
            void operator=( const RefCounted& ref )
            {
            }
            
        public:
            RefCounted() :
                m_refCount(0)
            {
            }
            
            void inc()
            {
                ++m_refCount;
            }
            
            bool dec()
            {
                return --m_refCount;
            }
    };
    
}

#endif
