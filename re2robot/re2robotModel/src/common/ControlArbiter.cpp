#include <re2robotModel/ControlArbiter.h>
#include <re2robotModel/ControlArbiterState.h>
#include <re2robotModel/Model.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( re2robotModel, ControlArbiter, re2::robot::ControlArbiter, re2::robot::Arbiter );

namespace re2
{
    namespace robot
    {
        
        ControlArbiter::ControlArbiter() :
            Super()
        {
        }
        
        ControlArbiter::~ControlArbiter()
        {
        }
        
        bool ControlArbiter::init()
        {
            m_cmdSub = model()->node().subscribe( "/re2/robot/model/arbiters/control/cmd", 1, &ControlArbiter::cmd, this );
            m_statePub = model()->node().advertise< re2robotModel::ControlArbiterState >( "/re2/robot/model/arbiters/control/state", 1, true );
            
            m_activeController = config().initialController;
            if ( !m_activeController.empty() )
            {
                m_controllers.insert( m_activeController );
            }
            sendState();
            return true;
        }
        
        void ControlArbiter::cmd( const re2robotModel::ControlArbiterCmd& msg )
        {
            switch ( msg.mode )
            {
                case re2robotModel::ControlArbiterCmd::MODE_ADD_CONTROLLER:
                {
                    m_controllers.insert( msg.controller );
                    sendState();
                    break;
                }
                case re2robotModel::ControlArbiterCmd::MODE_REMOVE_CONTROLLER:
                {
                    m_controllers.erase( msg.controller );
                    sendState();
                    break;
                }
                case re2robotModel::ControlArbiterCmd::MODE_SET_ACTIVE_CONTROLLER:
                {
                    m_activeController = msg.controller;
                    if ( !m_activeController.empty() )
                    {
                        m_controllers.insert( m_activeController );
                    }
                    sendState();
                    break;
                }
                default:
                {
                    ROS_ERROR( "unknown ControlArbiterCmd.mode %d", msg.mode );
                    break;
                }
            }
        }
        
        void ControlArbiter::sendState()
        {
            re2robotModel::ControlArbiterState msg;
            msg.activeController = m_activeController;
            for ( std::set< std::string >::const_iterator i = m_controllers.begin(); i != m_controllers.end(); ++i )
            {
                msg.controllers.push_back( *i );
            }
            m_statePub.publish( msg );
        }
        
        bool ControlArbiter::forward( const std::string& joint, const re2robotModel::JointCmd& cmd ) const
        {
            return cmd.controller == m_activeController;
        }
        
    }
}
