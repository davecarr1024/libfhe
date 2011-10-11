#include <re2robotDriver/DriverProxy.h>
#include <re2math/RE2Math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace re2;
using namespace robot;

boost::shared_ptr< DriverProxy > driver;

TEST( DriverTest, motion )
{
    static const double POS_THRESHOLD = 0.01, VEL_THRESHOLD = 0.01;
    
    ASSERT_TRUE( driver );
    
    DriveProxyPtr drive;
    for ( size_t i = 0; i < 10 && !drive; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
        drive = driver->getDrive( "test_sim_drive" );
    }
    ASSERT_TRUE( drive );
    
    ASSERT_EQ( "test_sim_drive", drive->config().name );
    ASSERT_EQ( "re2robotDriver/SimDrive", drive->config().type );
    
    double start = drive->state().position;
    for ( size_t i = 0; i < 20 && drive->state().position < start + drive->config().maxVelocity; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        drive->setEffort( 1 );
        ros::spinOnce();
    }
    ASSERT_GT( drive->state().position, start + drive->config().maxVelocity );
    
    for ( size_t i = 0; i < 20 && Math::abs( drive->state().velocity ) > VEL_THRESHOLD; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        drive->setEffort( 0 );
        ros::spinOnce();
    }
    ASSERT_NEAR( drive->state().velocity, 0, VEL_THRESHOLD );
    
    start = drive->state().position;
    for ( size_t i = 0; i < 20 && drive->state().position > start - 1; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        drive->setVelocity( -1 );
        ros::spinOnce();
    }
    ASSERT_LT( drive->state().position, start - 1 );
    
    for ( size_t i = 0; i < 20 && Math::abs( drive->state().velocity ) > VEL_THRESHOLD; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        drive->setEffort( 0 );
        ros::spinOnce();
    }
    ASSERT_NEAR( drive->state().velocity, 0, VEL_THRESHOLD );
    
    double dest = drive->state().position + 1;
    for ( size_t i = 0; i < 20 && Math::abs( drive->state().position - dest ) > POS_THRESHOLD; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        drive->setPosition( dest );
        ros::spinOnce();
    }
    ASSERT_NEAR( drive->state().position, dest, POS_THRESHOLD );
    
    ASSERT_TRUE( driver->removeDrive( "test_sim_drive" ) );
    ASSERT_FALSE( driver->getDrive( "test_sim_drive" ) );
}

int main( int argc, char** argv )
{
    ::testing::InitGoogleTest( &argc, argv );
    ros::init( argc, argv, "DriverTest" );
    ros::NodeHandle node;
    driver.reset( new DriverProxy( node ) );
    return RUN_ALL_TESTS();
}
