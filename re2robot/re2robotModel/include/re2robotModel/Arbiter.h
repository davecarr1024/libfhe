#ifndef RE2_ROBOT_MODEL_ARBITER_H
#define RE2_ROBOT_MODEL_ARBITER_H

#include <re2robotModel/ArbiterConfig.h>
#include <re2robotModel/JointCmd.h>
#include <re2robotModel/Transmission.h>

namespace re2
{
    namespace robot
    {
        
        class Model;
        
        class Arbiter : public boost::noncopyable
        {
            friend class Model;
            
            public:
                virtual ~Arbiter();
                
                Model* model() const;
                const re2robotModel::ArbiterConfig& config() const;
                
                virtual bool init();
                virtual void update();
                virtual void shutdown();
                
                virtual bool forward( const std::string& joint, const re2robotModel::JointCmd& cmd ) const = 0;
                
            protected:
                Arbiter();
                
            private:
                bool init( Model* model, const re2robotModel::ArbiterConfig& config );
                
                Model* m_model;
                re2robotModel::ArbiterConfig m_config;
                
                ros::Publisher m_configPub;
                
                ros::Timer m_timer;
                void update( const ros::TimerEvent& e );
        };
        
        typedef boost::shared_ptr< Arbiter > ArbiterPtr;
        
    }
}

#endif
