#include <re2robotModel/Transmission.h>
#include <re2robotModel/Model.h>
#include <re2robotModel/JointState.h>

namespace re2
{
    namespace robot
    {
        
        Transmission::Transmission()
        {
        }
        
        Transmission::~Transmission()
        {
        }
        
        bool Transmission::init( Model* model, const re2robotModel::TransmissionConfig& config )
        {
            ROS_ASSERT( m_model = model );
            m_config = config;
            
            m_timer = m_model->node().createTimer( ros::Duration( m_config.timerIntervalS ), &Transmission::update, this );
            m_configPub = m_model->node().advertise< re2robotModel::TransmissionConfig >( "/re2/robot/model/joints/" + m_config.joint + "/transmission_config", 1, true );
            m_statePub = m_model->node().advertise< re2robotModel::JointState >( "/re2/robot/model/joints/" + m_config.joint + "/state", 1, true );
            m_cmdSub = m_model->node().subscribe( "/re2/robot/model/joints/" + m_config.joint + "/cmd", 1, &Transmission::cmd, this );
            
            m_configPub.publish( m_config );
            
            return init();
        }
        
        Model* Transmission::model() const
        {
            return m_model;
        }
        
        const re2robotModel::TransmissionConfig& Transmission::config() const
        {
            return m_config;
        }
        
        DriveProxyPtr Transmission::drive() const
        {
            return m_model->driver().getDrive( config().drive );
        }
        
        boost::shared_ptr< const urdf::Joint > Transmission::joint() const
        {
            return m_model->model().getJoint( config().joint );
        }
        
        bool Transmission::init()
        {
            return true;
        }
        
        void Transmission::update()
        {
        }
        
        void Transmission::shutdown()
        {
        }
        
        void Transmission::update( const ros::TimerEvent& e )
        {
            if ( drive() && joint() )
            {
                update();
                sendState();
            }
        }
        
        void Transmission::sendState()
        {
            if ( drive() && joint() )
            {
                re2robotModel::JointState msg;
                msg.header.stamp = ros::Time::now();
                msg.effort = getEffort();
                msg.velocity = getVelocity();
                msg.position = getPosition();
                m_statePub.publish( msg );
            }
        }
        
        void Transmission::cmd( const re2robotModel::JointCmd& msg )
        {
            if ( drive() && joint() && model()->forward( config().joint, msg ) )
            {
                switch ( msg.mode )
                {
                    case re2robotModel::JointCmd::MODE_EFFORT:
                    {
                        setEffort( msg.cmd );
                        break;
                    }
                    case re2robotModel::JointCmd::MODE_VELOCITY:
                    {
                        setVelocity( msg.cmd );
                        break;
                    }
                    case re2robotModel::JointCmd::MODE_POSITION:
                    {
                        setPosition( msg.cmd );
                        break;
                    }
                    default:
                    {
                        ROS_ERROR( "invalid JointCmd.mode %d", msg.mode );
                        setEffort( 0 );
                        break;
                    }
                }
                sendState();
            }
        }
        
    }
}
