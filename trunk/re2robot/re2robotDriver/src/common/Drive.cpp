#include <re2robotDriver/Drive.h>
#include <re2robotDriver/DriveState.h>

namespace re2
{
    namespace robot
    {
        
        Drive::Drive() :
            m_mode( MODE_IDLE )
        {
        }
        
        bool Drive::init( const ros::NodeHandle& node, const re2robotDriver::DriveConfig& config )
        {
            m_node = node;
            m_config = config;
            m_configPub = m_node.advertise< re2robotDriver::DriveConfig >( "/re2/robot/driver/drives/" + m_config.name + "/config", 1, true );
            m_statePub = m_node.advertise< re2robotDriver::DriveState >( "/re2/robot/driver/drives/" + m_config.name + "/state", 1, true );
            m_cmdSub = m_node.subscribe( "/re2/robot/driver/drives/" + m_config.name + "/cmd", 1, &Drive::cmd, this );
            m_timer = m_node.createTimer( ros::Duration( m_config.updateIntervalS ), &Drive::update, this );
            m_configPub.publish( m_config );
            return init();
        }
        
        Drive::~Drive()
        {
        }
        
        const re2robotDriver::DriveConfig& Drive::config() const
        {
            return m_config;
        }
        
        bool Drive::init()
        {
            return true;
        }
        
        void Drive::update()
        {
        }
        
        void Drive::shutdown()
        {
        }
        
        void Drive::sendState()
        {
            re2robotDriver::DriveState msg;
            msg.header.stamp = ros::Time::now();
            msg.effort = getEffort();
            msg.velocity = getVelocity();
            msg.position = getPosition();
            m_statePub.publish( msg );
        }
        
        void Drive::cmd( const re2robotDriver::DriveCmd& msg )
        {
            switch ( msg.mode )
            {
                case re2robotDriver::DriveCmd::MODE_EFFORT:
                {
                    m_mode = MODE_EFFORT;
                    m_cmd = msg.cmd;
                    m_lastCmd = ros::Time::now();
                    break;
                }
                case re2robotDriver::DriveCmd::MODE_VELOCITY:
                {
                    m_mode = MODE_VELOCITY;
                    m_cmd = msg.cmd;
                    m_lastCmd = ros::Time::now();
                    break;
                }
                case re2robotDriver::DriveCmd::MODE_POSITION:
                {
                    m_mode = MODE_POSITION;
                    m_cmd = msg.cmd;
                    m_lastCmd = ros::Time::now();
                    break;
                }
                default:
                {
                    ROS_ERROR( "invalid drive cmd mode %d", msg.mode );
                    break;
                }
            }
            
            consumeCmd();
            sendState();
        }
        
        void Drive::update( const ros::TimerEvent& e )
        {
            ros::Time now = ros::Time::now();
            double dt = ( now - m_lastCmd ).toSec();
            if ( m_mode != MODE_IDLE && dt > config().cmdTimeoutS )
            {
                ROS_WARN( "drive %s cmd timeout %f > %f", config().name.c_str(), dt, config().cmdTimeoutS );
                m_mode = MODE_IDLE;
            }
            
            update();
            consumeCmd();
            sendState();
        }
        
        void Drive::consumeCmd()
        {
            switch ( m_mode )
            {
                case MODE_IDLE:
                {
                    setEffort( 0 );
                    break;
                }
                case MODE_EFFORT:
                {
                    setEffort( m_cmd );
                    break;
                }
                case MODE_VELOCITY:
                {
                    setVelocity( m_cmd );
                    break;
                }
                case MODE_POSITION:
                {
                    setPosition( m_cmd );
                    break;
                }
            }
        }
        
    }
}
