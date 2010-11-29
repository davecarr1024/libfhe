extern "C"
{
#include <fhe/mat3.h>
}
#include <gtest/gtest.h>
#include <math.h>

static const double EPS = 1e-5;

TEST( math_test, mat3 )
{
    fhe_vec2_t test, mtest, v, t, s;
    double r;
    fhe_mat3_t mt, mr, ms, m, imt, imr, ims;
    
    fhe_vec2_init( &v, 10, 15 );
    
    //t + v == mt * v
    
    fhe_vec2_init( &t, -5, 12 );
    fhe_vec2_add( &test, &v, &t );
    ASSERT_NEAR( test.x, 5, EPS );
    ASSERT_NEAR( test.y, 27, EPS );
    
    fhe_mat3_init_translation( mt, &t );
    fhe_mat3_mul_vec2( &test, mt, &v );
    ASSERT_NEAR( test.x, 5, EPS );
    ASSERT_NEAR( test.y, 27, EPS );
    
    //r * v == mr * v

    r = M_PI / 2.0;
    fhe_vec2_rotate( &test, &v, r );
    ASSERT_NEAR( test.x, -15, EPS );
    ASSERT_NEAR( test.y, 10, EPS );
    
    fhe_mat3_init_rotation( mr, r );
    fhe_mat3_mul_vec2( &test, mr, &v );
    ASSERT_NEAR( test.x, -15, EPS );
    ASSERT_NEAR( test.y, 10, EPS );
    
    //s * v == ms * v
    
    fhe_vec2_init( &s, -1, 2 );
    fhe_vec2_mul_vec2( &test, &v, &s );
    ASSERT_NEAR( test.x, -10, EPS );
    ASSERT_NEAR( test.y, 30, EPS );
    
    fhe_mat3_init_scale( ms, &s );
    fhe_mat3_mul_vec2( &test, ms, &v );
    ASSERT_NEAR( test.x, -10, EPS );
    ASSERT_NEAR( test.y, 30, EPS );
    
    //s * ( r * ( v + t ) ) == ms * ( mr * ( mt * v ) ) == ( ms * mr * mt ) * v
    
    fhe_vec2_copy( &test, &v );
    fhe_vec2_iadd( &test, &t );
    fhe_vec2_irotate( &test, r );
    fhe_vec2_imul_vec2( &test, &s );
    
    fhe_vec2_copy( &mtest, &v );
    fhe_mat3_imul_vec2( &mtest, mt );
    fhe_mat3_imul_vec2( &mtest, mr );
    fhe_mat3_imul_vec2( &mtest, ms );
    
    ASSERT_NEAR( test.x, mtest.x, EPS );
    ASSERT_NEAR( test.y, mtest.y, EPS );
    
    fhe_mat3_copy( m, FHE_MAT3_I );
    fhe_mat3_imul_mat3( m, ms );
    fhe_mat3_imul_mat3( m, mr );
    fhe_mat3_imul_mat3( m, mt );
    fhe_mat3_mul_vec2( &mtest, m, &v );

    ASSERT_NEAR( test.x, mtest.x, EPS );
    ASSERT_NEAR( test.y, mtest.y, EPS );
    
    //( ( v - t ) * -r ) / s == ims * ( imr * ( imt * v ) ) == ( ims * imr * imt ) * v
    
    fhe_vec2_copy( &test, &v );
    fhe_vec2_isub( &test, &t );
    fhe_vec2_irotate( &test, -r );
    fhe_vec2_idiv_vec2( &test, &s );
    
    fhe_mat3_inverse( imt, mt );
    fhe_mat3_inverse( imr, mr );
    fhe_mat3_inverse( ims, ms );
    
    fhe_vec2_copy( &mtest, &v );
    fhe_mat3_imul_vec2( &mtest, imt );
    fhe_mat3_imul_vec2( &mtest, imr );
    fhe_mat3_imul_vec2( &mtest, ims );
    
    ASSERT_NEAR( test.x, mtest.x, EPS );
    ASSERT_NEAR( test.y, mtest.y, EPS );
    
    fhe_mat3_copy( m, FHE_MAT3_I );
    fhe_mat3_imul_mat3( m, ims );
    fhe_mat3_imul_mat3( m, imr );
    fhe_mat3_imul_mat3( m, imt );
    fhe_mat3_mul_vec2( &mtest, m, &v );

    ASSERT_NEAR( test.x, mtest.x, EPS );
    ASSERT_NEAR( test.y, mtest.y, EPS );
    
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
