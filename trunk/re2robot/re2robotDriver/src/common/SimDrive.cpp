#include <re2robotDriver/SimDrive.h>
#include <re2math/RE2Math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( re2robotDriver, SimDrive, re2::robot::SimDrive, re2::robot::Drive );

namespace re2
{
    namespace robot
    {
        
        SimDrive::SimDrive() :
            Super()
        {
        }
        
        SimDrive::~SimDrive()
        {
        }
        
        bool SimDrive::init()
        {
            m_position = config().initialPosition;
            m_velocity = 0;
            m_mode = MODE_EFFORT;
            m_cmd = 0;
            m_lastUpdate = ros::Time::now();
            
            return true;
        }
        
        void SimDrive::update()
        {
            ros::Time now = ros::Time::now();
            double dt = ( now - m_lastUpdate ).toSec();
            m_lastUpdate = now;
            
            double cmdVel = 0;
            switch ( m_mode )
            {
                case MODE_EFFORT:
                {
                    cmdVel = m_cmd * config().maxVelocity;
                    break;
                }
                case MODE_VELOCITY:
                {
                    cmdVel = m_cmd;
                    break;
                }
                case MODE_POSITION:
                {
                    cmdVel = Math::clamp( ( m_cmd - m_position ) * config().positionPGain - m_velocity * config().positionDGain, -config().maxVelocity, config().maxVelocity );
                    break;
                }
            }
            
            m_velocity = Math::clamp( m_velocity + ( cmdVel - m_velocity ) / config().velocityTimeToTargetS * dt, -config().maxVelocity, config().maxVelocity );
            m_position = Math::clamp( m_position + m_velocity * dt, config().minPosition, config().maxPosition );
        }
        
        double SimDrive::getEffort() const
        {
            return m_velocity / config().maxVelocity;
        }
        
        double SimDrive::getVelocity() const
        {
            return m_velocity;
        }
        
        double SimDrive::getPosition() const
        {
            return m_position;
        }
        
        void SimDrive::setEffort( double effort )
        {
            m_cmd = effort;
            m_mode = MODE_EFFORT;
        }
        
        void SimDrive::setVelocity( double velocity )
        {
            m_cmd = velocity;
            m_mode = MODE_VELOCITY;
        }
        
        void SimDrive::setPosition( double position )
        {
            m_cmd = position;
            m_mode = MODE_POSITION;
        }
        
    }
}
