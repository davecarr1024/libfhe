#ifndef MSGDELEGATE_H
#define MSGDELEGATE_H

#include "IMsgDelegate.h"

#include <Poco/AutoPtr.h>
#include <Poco/Delegate.h>

namespace SGE
{
    template <class TObj, class TArgs>
    class MsgDelegate : public IMsgDelegate
    {
        public:
            typedef Poco::AutoPtr<TObj> TObjPtr;
            typedef void (TObj::*NotifyMethod)(const TArgs&);
            
        private:
            TObjPtr m_obj;
            NotifyMethod m_method;
            
        public:
            MsgDelegate(TObjPtr obj, NotifyMethod method) :
                m_obj(obj),
                m_method(method)
            {
            }
            
            MsgDelegate(const MsgDelegate& delegate) :
                m_obj(delegate.m_obj),
                m_method(delegate.m_method)
            {
            }
            
            MsgDelegate& operator=(const MsgDelegate& delegate)
            {
                m_obj = delegate.m_obj;
                m_method = delegate.m_method;
                return *this;
            }
            
            void notify(const Notification& args)
            {
                (m_obj->*m_method)(static_cast<const TArgs&>(args));
            }
    };
    
    #define SUBSCRIBE_MSG( tobj, targs ) subscribe< targs > (new MsgDelegate< tobj, targs > (this, &tobj::on##targs ));
    #define ON_MSG( targs, tvar ) void on##targs ( const targs& tvar)
    #define ON_MSG_DECL( targs ) void on##targs ( const targs& msg);
    #define ON_MSG_IMPL( tobj, targs, tvar) void tobj::on##targs ( const targs& tvar)
    
}

#endif
