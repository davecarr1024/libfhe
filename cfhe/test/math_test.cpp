#include <gtest/gtest.h>
extern "C"
{
#include <fhe/mat3.h>
}

static const double EPS = 1e-5;

TEST( math_test, mat3 )
{
    fhe_vec2_t test, v, t;
    fhe_mat3_t mt;
    
    fhe_vec2_init( &v, 10, 15 );
    fhe_vec2_init( &t, -5, 12 );
    fhe_vec2_add( &test, &v, &t );
    ASSERT_NEAR( test.x, 5, EPS );
    ASSERT_NEAR( test.y, 27, EPS );
    
    fhe_mat3_init_translation( mt, &t );
    fhe_mat3_mul_vec2( &test, mt, &v );
    ASSERT_NEAR( test.x, 5, EPS );
    ASSERT_NEAR( test.y, 27, EPS );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
