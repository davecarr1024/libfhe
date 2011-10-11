#ifndef RE2_ROBOT_MODEL_CONTROL_ARBITER_H
#define RE2_ROBOT_MODEL_CONTROL_ARBITER_H

#include <re2robotModel/Arbiter.h>
#include <re2robotModel/ControlArbiterCmd.h>
#include <Poco/MetaObject.h>

namespace re2
{
    namespace robot
    {

        class ControlArbiter : public Arbiter
        {
            typedef Arbiter Super;
            friend class Poco::MetaObject< ControlArbiter, Arbiter >;
            
            public:
                virtual ~ControlArbiter();
                
                bool init();
                
                bool forward( const std::string& joint, const re2robotModel::JointCmd& cmd ) const;
                
            private:
                ControlArbiter();
                
                std::string m_activeController;
                std::set< std::string > m_controllers;
                
                ros::Subscriber m_cmdSub;
                void cmd( const re2robotModel::ControlArbiterCmd& msg );
                
                ros::Publisher m_statePub;
                void sendState();
        };
        
    }
}

#endif
