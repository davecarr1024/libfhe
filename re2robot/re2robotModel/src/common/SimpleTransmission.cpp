#include <re2robotModel/SimpleTransmission.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( re2robotModel, SimpleTransmission, re2::robot::SimpleTransmission, re2::robot::Transmission );

namespace re2
{
    namespace robot
    {
        
        SimpleTransmission::SimpleTransmission() :
            Super()
        {
        }
        
        SimpleTransmission::~SimpleTransmission()
        {
        }
        
        double SimpleTransmission::getEffort() const
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                return drive->state().effort;
            }
            else
            {
                return 0;
            }
        }
        
        double SimpleTransmission::getVelocity() const
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                return drive->state().velocity;
            }
            else
            {
                return 0;
            }
        }
        
        double SimpleTransmission::getPosition() const
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                return drive->state().position;
            }
            else
            {
                return 0;
            }
        }
        
        void SimpleTransmission::setEffort( double effort )
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                drive->setEffort( effort );
            }
        }
        
        void SimpleTransmission::setVelocity( double velocity )
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                drive->setVelocity( velocity );
            }
        }
        
        void SimpleTransmission::setPosition( double position )
        {
            DriveProxyPtr drive = this->drive();
            if ( drive )
            {
                drive->setPosition( position );
            }
        }
        
    }
}
