#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <gtest/gtest.h>

class Node;

class IFunc
{
    public:
        virtual ~IFunc() {}
};

typedef boost::shared_ptr< IFunc > IFuncPtr;

template <class TArg0, class TArg1>
class TFunc;

template <>
class TFunc<void,void> : public IFunc
{
    public:
        virtual void call() = 0;
};

template <class TArg0>
class TFunc<TArg0,void> : public IFunc
{
    public:
        virtual void call( TArg0 arg0 ) = 0;
};

template <class TObj, class TArg0, class TArg1>
class Func;

template <class TObj>
class Func<TObj,void,void> : public TFunc<void,void>
{
    public:
        typedef void (TObj::*Ptr)();
        
    private:
        Node* m_object;
        Ptr m_ptr;
        
    public:
        Func( Node* object, Ptr ptr ) :
            m_object( object ),
            m_ptr( ptr )
        {
        }
        
        void call()
        {
            if ( TObj* t = dynamic_cast<TObj*>( m_object ) )
            {
                (t->*m_ptr)();
            }
        }
};

template <class TObj,class TArg0>
class Func<TObj,TArg0,void> : public TFunc<TArg0,void>
{
    public:
        typedef void (TObj::*Ptr)( TArg0 arg );
        
    private:
        Node* m_object;
        Ptr m_ptr;
        
    public:
        Func( Node* object, Ptr ptr ) :
            m_object( object ),
            m_ptr( ptr )
        {
        }
        
        void call( TArg0 arg0 )
        {
            if ( TObj* t = dynamic_cast<TObj*>( m_object ) )
            {
                (t->*m_ptr)( arg0 );
            }
        }
};

class Node
{
    private:
        std::map< std::string, IFuncPtr > m_funcs;
        
        void addFunc( const std::string& name, IFuncPtr func )
        {
            m_funcs[name] = func;
        }
        
    public:
        void call()
        {
        }
};

class IFuncDesc
{
    public:
        virtual IFunc* build( Node* object ) = 0;
};

typedef boost::shared_ptr< IFuncDesc > IFuncDescPtr;

template <class TObj, class TArg0, class TArg1>
class FuncDesc;

template <class TObj>
class FuncDesc<TObj,void,void>
{
    public:
        typedef void (TObj::*Ptr)();
        
    private:
        Ptr m_ptr;
        
    public:
        FuncDesc( Ptr ptr ) :
            m_ptr( ptr )
        {
        }
        
        IFunc* build( Node* object )
        {
            return new Func<TObj,void,void>( object, m_ptr );
        }
};

template <class TObj, class TArg0>
class FuncDesc<TObj,TArg0,void>
{
    public:
        typedef void (TObj::*Ptr)( TArg0 arg0 );
        
    private:
        Ptr m_ptr;
        
    public:
        FuncDesc( Ptr ptr ) :
            m_ptr( ptr )
        {
        }
        
        IFunc* build( Node* object )
        {
            return new Func<TObj,TArg0,void>( object, m_ptr );
        }
};

class INodeDesc
{
    public:
        virtual Node* build() = 0;
};

typedef boost::shared_ptr< INodeDesc > INodeDescPtr;

template <class T>
class NodeDesc : public INodeDesc
{
    private:
        std::map< std::string, IFuncDescPtr > m_funcs;
        
        void addFunc( const std::string& name, IFuncDescPtr func )
        {
            m_funcs[name] = func;
        }
        
    public:
        Node* build()
        {
            Node* n = new T;
            for ( std::map< std::string, IFuncDescPtr >::const_iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
            {
                n->addFunc( i->first, IFuncPtr( i->second->build( n ) ) );
            }
            return n;
        }
};

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
