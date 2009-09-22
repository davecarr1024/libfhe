#ifndef IMSGDELEGATE_H
#define IMSGDELEGATE_H

#include "Notification.h"

#include <string>
#include <map>

#include <Poco/SharedPtr.h>

namespace SGE
{
    class IMsgDelegate;
    
    typedef Poco::SharedPtr<IMsgDelegate> MsgDelegatePtr;
    typedef std::map<std::string, MsgDelegatePtr> MsgDelegateMap;
    
    class IMsgDelegate 
    {
        public:
            virtual void notify(const Notification& notification)=0;
    };
    
}

#endif
