#ifndef RE2_ROBOT_MODEL_MODEL_H
#define RE2_ROBOT_MODEL_MODEL_H

#include <re2robotModel/Transmission.h>
#include <re2robotModel/Arbiter.h>
#include <re2robotModel/AddTransmission.h>
#include <re2robotModel/RemoveTransmission.h>
#include <re2robotModel/AddArbiter.h>
#include <re2robotModel/RemoveArbiter.h>
#include <re2robotModel/ControlCmd.h>
#include <re2robotDriver/DriverProxy.h>
#include <urdf/model.h>
#include <pluginlib/class_loader.h>

namespace re2
{
    namespace robot
    {
        
        class Model : public boost::noncopyable
        {
            public:
                Model( const ros::NodeHandle& node, const std::string& robotDescPath = "/robot_description" );
                virtual ~Model();

                ros::NodeHandle& node();
                DriverProxy& driver();
                urdf::Model& model();
                
                TransmissionPtr addTransmission( const re2robotModel::TransmissionConfig& config );
                bool removeTransmission( const std::string& joint );
                
                TransmissionPtr getTransmission( const std::string& joint ) const;
                
                typedef std::map< std::string, TransmissionPtr >::const_iterator TransmissionIterator;
                TransmissionIterator transmissionsBegin() const;
                TransmissionIterator transmissionsEnd() const;
                
                ArbiterPtr addArbiter( const re2robotModel::ArbiterConfig& config );
                bool removeArbiter( const std::string& type );
                
                bool forward( const std::string& joint, const re2robotModel::JointCmd& cmd ) const;
                
            private:
                ros::NodeHandle m_node;
                DriverProxy m_driver;
                urdf::Model m_model;
                pluginlib::ClassLoader< Transmission > m_transmissionLoader;
                pluginlib::ClassLoader< Arbiter > m_arbiterLoader;
                std::map< std::string, TransmissionPtr > m_transmissions;
                std::map< std::string, ArbiterPtr > m_arbiters;
                
                ros::ServiceServer m_addTransmissionSrv;
                bool addTransmission( re2robotModel::AddTransmission::Request& req,
                                      re2robotModel::AddTransmission::Response& res );
                                      
                ros::ServiceServer m_removeTransmissionSrv;
                bool removeTransmission( re2robotModel::RemoveTransmission::Request& req,
                                         re2robotModel::RemoveTransmission::Response& res );
                                         
                ros::ServiceServer m_addArbiterSrv;
                bool addArbiter( re2robotModel::AddArbiter::Request& req,
                                 re2robotModel::AddArbiter::Response& res );
                                      
                ros::ServiceServer m_removeArbiterSrv;
                bool removeArbiter( re2robotModel::RemoveArbiter::Request& req,
                                    re2robotModel::RemoveArbiter::Response& res );
                                         
                ros::Publisher m_configPub;
                void sendConfig();
        };
        
    }
}

#endif
