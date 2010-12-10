#include <fhe/Vec.h>
#include <fhe/Vec.h>
#include <gtest/gtest.h>
using namespace fhe;

TEST( math_test, vec )
{
    ASSERT_TRUE( Vec<3>( 0, 1, 2 ).equals( Vec<3>( 0, 1, 2 ) ) );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
