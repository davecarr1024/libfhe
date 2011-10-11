#include <re2robotUtil/Config.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace re2::robot;

TEST( ConfigTest, load )
{
    Config config,
           robot = config[ "robot" ],
           joints = robot[ "joint" ];
    ASSERT_EQ( 2, joints.size() );
    ASSERT_EQ( "world", joints[ 0 ][ "parent" ][ "link" ].to<std::string>( "" ) );
}

int main( int argc, char** argv )
{
    ::testing::InitGoogleTest( &argc, argv );
    ros::init( argc, argv, "ConfigTest" );
    ros::NodeHandle node;
    return RUN_ALL_TESTS();
}
