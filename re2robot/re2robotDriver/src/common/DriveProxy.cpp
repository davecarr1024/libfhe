#include <re2robotDriver/DriveProxy.h>
#include <re2robotDriver/DriveCmd.h>

namespace re2
{
    namespace robot
    {
        
        DriveProxy::DriveProxy( const ros::NodeHandle& node, const re2robotDriver::DriveConfig& config ) :
            m_node( node ),
            m_config( config ),
            m_stateSub( m_node.subscribe( "/re2/robot/driver/drives/" + m_config.name + "/state", 1, &DriveProxy::state, this ) ),
            m_cmdPub( m_node.advertise< re2robotDriver::DriveCmd >( "/re2/robot/driver/drives/" + m_config.name + "/cmd", 1 ) )
        {
        }
        
        DriveProxy::~DriveProxy()
        {
        }
        
        const re2robotDriver::DriveConfig& DriveProxy::config() const
        {
            return m_config;
        }
        
        const re2robotDriver::DriveState& DriveProxy::state() const
        {
            return m_state;
        }
        
        void DriveProxy::setEffort( double effort )
        {
            re2robotDriver::DriveCmd msg;
            msg.mode = re2robotDriver::DriveCmd::MODE_EFFORT;
            msg.cmd = effort;
            m_cmdPub.publish( msg );
        }
        
        void DriveProxy::setVelocity( double velocity )
        {
            re2robotDriver::DriveCmd msg;
            msg.mode = re2robotDriver::DriveCmd::MODE_VELOCITY;
            msg.cmd = velocity;
            m_cmdPub.publish( msg );
        }
        
        void DriveProxy::setPosition( double position )
        {
            re2robotDriver::DriveCmd msg;
            msg.mode = re2robotDriver::DriveCmd::MODE_POSITION;
            msg.cmd = position;
            m_cmdPub.publish( msg );
        }
        
        void DriveProxy::state( const re2robotDriver::DriveState& msg )
        {
            m_state = msg;
        }
        
    }
}
