#ifndef RE2_ROBOT_MODEL_TRANSMISSION_H
#define RE2_ROBOT_MODEL_TRANSMISSION_H

#include <re2robotModel/TransmissionConfig.h>
#include <re2robotModel/JointCmd.h>
#include <re2robotDriver/DriveProxy.h>
#include <urdf/joint.h>

namespace re2
{
    namespace robot
    {
        
        class Model;
        
        class Transmission : public boost::noncopyable
        {
            friend class Model;
            
            public:
                virtual ~Transmission();
                
                Model* model() const;
                const re2robotModel::TransmissionConfig& config() const;
                DriveProxyPtr drive() const;
                boost::shared_ptr< const urdf::Joint > joint() const;
                
                virtual bool init();
                virtual void update();
                virtual void shutdown();
                
                virtual double getEffort() const = 0;
                virtual double getVelocity() const = 0;
                virtual double getPosition() const = 0;
                
                virtual void setEffort( double effort ) = 0;
                virtual void setVelocity( double velocity ) = 0;
                virtual void setPosition( double position ) = 0;
                
            protected:
                Transmission();
                
            private:
                Model* m_model;
                re2robotModel::TransmissionConfig m_config;
                
                bool init( Model* model, const re2robotModel::TransmissionConfig& config );
                
                ros::Timer m_timer;
                void update( const ros::TimerEvent& e );
                
                ros::Publisher m_configPub;
                
                ros::Publisher m_statePub;
                void sendState();
                
                ros::Subscriber m_cmdSub;
                void cmd( const re2robotModel::JointCmd& msg );
        };
        
        typedef boost::shared_ptr< Transmission > TransmissionPtr;
        
    }
}

#endif
