#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <gtest/gtest.h>

class Object;

template <class TArg0, class TArg1>
class TFunc;

class IFunc
{
    public:
        virtual ~IFunc() {}
};

typedef boost::shared_ptr< IFunc > IFuncPtr;

template <>
class TFunc<void,void> : public IFunc
{
    public:
        virtual void call( Object* obj ) = 0;
};

template <class TArg0>
class TFunc<TArg0,void> : public IFunc
{
    public:
        virtual void call( Object* obj, TArg0 arg0 ) = 0;
};

template <class TObj, class TArg0, class TArg1>
class Func;

template <class TObj>
class Func<TObj, void, void> : public TFunc<void,void>
{
    public:
        typedef void (TObj::*Ptr)();
        
    private:
        Ptr m_func;
        
    public:
        Func( Ptr func ) :
            m_func( func )
        {
        }
        
        void call( Object* obj )
        {
            if ( TObj* t = dynamic_cast<TObj*>( obj ) )
            {
                (t->*m_func)();
            }
        }
};

template <class TObj, class TArg0>
class Func<TObj, TArg0, void> : public TFunc<TArg0,void>
{
    public:
        typedef void (TObj::*Ptr)( TArg0 arg0 );
        
    private:
        Ptr m_func;
        
    public:
        Func( Ptr func ) :
            m_func( func )
        {
        }
        
        void call( Object* obj, TArg0 arg0 )
        {
            if ( TObj* t = dynamic_cast<TObj*>( obj ) )
            {
                (t->*m_func)( arg0 );
            }
        }
};

class Object
{
    private:
        std::map< std::string, IFuncPtr > m_funcs;
        
    public:
        virtual ~Object()
        {
        }
        
        template <class TObj>
        void func( const std::string& name, void (TObj::*func)() )
        {
            m_funcs[name] = IFuncPtr( new Func<TObj,void,void>( func ) );
        }
        
        template <class TObj, class TArg0>
        void func( const std::string& name, void (TObj::*func)( TArg0 arg0 ) )
        {
            m_funcs[name] = IFuncPtr( new Func<TObj,TArg0,void>( func ) );
        }

        void call( const std::string& name )
        {
            std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
            if ( i != m_funcs.end() )
            {
                if ( TFunc<void,void>* t = dynamic_cast<TFunc<void,void>*>( i->second.get() ) )
                {
                    t->call( this );
                }
            }
        }
        
        template <class TArg0>
        void call( const std::string& name, TArg0 arg0 )
        {
            std::map< std::string, IFuncPtr >::iterator i = m_funcs.find( name );
            if ( i != m_funcs.end() )
            {
                if ( TFunc<TArg0,void>* t = dynamic_cast<TFunc<TArg0,void>*>( i->second.get() ) )
                {
                    t->call( this, arg0 );
                }
            }
        }
};

class TestObject : public Object
{
    public:
        int m_i;
        
        void foo( int i )
        {
            m_i = i;
        }
        
        bool m_bar;
        
        void bar()
        {
            m_bar = true;
        }
        
        TestObject()
        {
            func( "bar", &TestObject::bar );
            func( "foo", &TestObject::foo );
        }
};

TEST( foo, foo )
{
    TestObject test;
    
    test.m_bar = false;
    test.call( "bar" );
    ASSERT_TRUE( test.m_bar );
    
    test.call( "foo" );
    
    test.m_i = 0;
    test.call( "foo", 1 );
    ASSERT_EQ( 1, test.m_i );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
