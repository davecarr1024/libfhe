#include <fhe/core/Vec.h>
#include <fhe/core/Rot.h>
#include <fhe/core/Mat.h>
#include <gtest/gtest.h>
using namespace fhe;

TEST( math_test, vec2 )
{
    Vec2d v( 1, 2 );
    Vec2d t( 2, 4 );
    Vec2d s( -1, 2 );
    double d = 5;
    
    ASSERT_EQ( v, v );
    ASSERT_EQ( Vec2d( -1, -2 ), -v );
    ASSERT_EQ( Vec2d( 3, 6 ), v + t );
    ASSERT_EQ( Vec2d( -1, -2 ), v - t );
    ASSERT_EQ( Vec2d( -1, 4 ), v * s );
    ASSERT_EQ( Vec2d( -1, 1 ), v / s );
    ASSERT_EQ( Vec2d( 5, 10 ), v * d );
    ASSERT_EQ( Vec2d( 0.2, 0.4 ), v / d );
    
    {
        Vec2d v( 1, 2 );
        v += t;
        ASSERT_EQ( Vec2d( 3, 6 ), v );
    }
    {
        Vec2d v( 1, 2 );
        v -= t;
        ASSERT_EQ( Vec2d( -1, -2 ), v );
    }
    {
        Vec2d v( 1, 2 );
        v *= s;
        ASSERT_EQ( Vec2d( -1, 4 ), v );
    }
    {
        Vec2d v( 1, 2 );
        v /= s;
        ASSERT_EQ( Vec2d( -1, 1 ), v );
    }
    {
        Vec2d v( 1, 2 );
        v *= d;
        ASSERT_EQ( Vec2d( 5, 10 ), v );
    }
    {
        Vec2d v( 1, 2 );
        v /= d;
        ASSERT_EQ( Vec2d( 0.2, 0.4 ), v );
    }
    
    ASSERT_EQ( 5, Vec2d( 3, 4 ).length() );
    ASSERT_EQ( Vec2d( 0, 1 ), Vec2d( 0, 10 ).norm() );
    ASSERT_EQ( 7, Vec2d( 2, 3 ).dot( Vec2d( 5, -1 ) ) );
    ASSERT_EQ( Vec2d( 3, 6 ), Vec2d( 1, 2 ).lerp( Vec2d( 5, 10 ), 0.5 ) );
    
    Rot2 r = Rot2::fromDegrees( 90 );
    ASSERT_EQ( Vec2d( 0, 1.5 ), Vec2d( r, 1.5 ) );
    ASSERT_EQ( Rot2::fromDegrees( 90 ), Vec2d::UNIT_X.getRotTo( Vec2d::UNIT_Y ) );
}

TEST( math_test, vec3 )
{
    Vec3d v( 1, 2, 3 );
    Vec3d t( 2, 4, 6 );
    Vec3d s( -1, 2, -2 );
    double d = 10;
    
    ASSERT_EQ( v, v );
    ASSERT_EQ( Vec3d( -1, -2, -3 ), -v );
    ASSERT_EQ( Vec3d( 3, 6, 9 ), v + t );
    ASSERT_EQ( Vec3d( -1, -2, -3 ), v - t );
    ASSERT_EQ( Vec3d( -1, 4, -6 ), v * s );
    ASSERT_EQ( Vec3d( -1, 1, -1.5 ), v / s );
    ASSERT_EQ( Vec3d( 10, 20, 30 ), v * d );
    ASSERT_EQ( Vec3d( 0.1, 0.2, 0.3 ), v / d );
    
    {
        Vec3d v( 1, 2, 3 );
        v += t;
        ASSERT_EQ( Vec3d( 3, 6, 9 ), v );
    }
    {
        Vec3d v( 1, 2, 3 );
        v -= t;
        ASSERT_EQ( Vec3d( -1, -2, -3 ), v );
    }
    {
        Vec3d v( 1, 2, 3 );
        v *= s;
        ASSERT_EQ( Vec3d( -1, 4, -6 ), v );
    }
    {
        Vec3d v( 1, 2, 3 );
        v /= s;
        ASSERT_EQ( Vec3d( -1, 1, -1.5 ), v );
    }
    {
        Vec3d v( 1, 2, 3 );
        v *= d;
        ASSERT_EQ( Vec3d( 10, 20, 30 ), v );
    }
    {
        Vec3d v( 1, 2, 3 );
        v /= d;
        ASSERT_EQ( Vec3d( 0.1, 0.2, 0.3 ), v );
    }
    
    ASSERT_EQ( Math::sqrt( 1 * 1 + 2 * 2 + 3 * 3 ), v.length() );
    ASSERT_EQ( 20, Vec3d( 2, 3, 4 ).dot( Vec3d( 1, 2, 3 ) ) );
    ASSERT_EQ( Vec3d::UNIT_Z, Vec3d::UNIT_X.cross( Vec3d::UNIT_Y ) );
    ASSERT_EQ( Vec3d::UNIT_X, Vec3d::UNIT_X.project( Vec3d( 1, 20, 100 ) ) );
    ASSERT_EQ( Vec3d( 3, 6, 6 ), Vec3d( 1, 2, 3 ).lerp( Vec3d( 5, 10, 9 ), 0.5 ) );
    
    Rot3 r( Vec3d::UNIT_Z, Math::PI/2 );
    ASSERT_EQ( Vec3d::UNIT_Y, Vec3d( r ) );
    ASSERT_EQ( r, Vec3d::UNIT_X.getRotTo( Vec3d::UNIT_Y ) );
}

TEST( math_test, rot2 )
{
    Rot2 r = Rot2::fromDegrees( 35 );
    Rot2 t = Rot2::fromDegrees( 75 );
    double d = 2;
    
    ASSERT_EQ( r, r );
    ASSERT_EQ( Rot2::fromRadians( Math::radians( 35 ) ), r );
    ASSERT_EQ( Math::radians( 35 ), r.radians() );
    ASSERT_EQ( Rot2::fromDegrees( 110 ), r + t );
    ASSERT_EQ( Rot2::fromDegrees( -40 ), r - t );
    ASSERT_EQ( Rot2::fromDegrees( 70 ), r * d );
    ASSERT_EQ( Rot2::fromDegrees( 17.5 ), r / d );
    ASSERT_EQ( Rot2::fromDegrees( -35 ), r.inverse() );
    ASSERT_EQ( r, ( r + Rot2::fromDegrees( 720 ) ).norm() );
    ASSERT_EQ( Vec2d::UNIT_Y, Rot2::fromDegrees( 90 ) * Vec2d::UNIT_X );
}

TEST( math_test, rot3 )
{
    ASSERT_EQ( Rot3( Vec3d::UNIT_Z, Math::PI/2 ), Rot3( Vec3d::UNIT_Y ) );
    
    Vec3d axis = Vec3d( 4, 5, 6 ).norm();
    double angle = 1.2;
    Rot3 r( axis, angle );
    
    ASSERT_EQ( r, r );
    
    Vec3d taxis;
    double tangle;
    r.toAxisAngle( taxis, tangle );
    ASSERT_EQ( axis, taxis );
    ASSERT_EQ( angle, tangle );
    
    ASSERT_EQ( Rot3( axis, angle * 2 ), r * r );
    ASSERT_EQ( Vec3d::UNIT_Y, Rot3( Vec3d::UNIT_Z, Math::PI/2 ) * Vec3d::UNIT_X );
    ASSERT_EQ( Rot3( axis, -angle ), r.inverse() );
}

template <size_t dim, typename T>
void mat_test( const Vec<dim,T>& v, const Vec<dim,T>& t, const Rot<dim>& r, const Vec<dim,T>& s )
{
    Mat<dim> mt = Mat<dim>::translation( t );
    Mat<dim> mr = Mat<dim>::rotation( r );
    Mat<dim> ms = Mat<dim>::scale( s );
    
    ASSERT_EQ( v + t, mt * v );
    ASSERT_EQ( r * v, mr * v );
    ASSERT_EQ( v * s, ms * v );
    ASSERT_EQ( s * ( r * ( t + v ) ), ms * ( mr * ( mt * v ) ) );
    ASSERT_EQ( s * ( r * ( t + v ) ), ( ms * mr * mt ) * v );
    
    Rot<dim> ri = r.inverse();
    
    Mat<dim> mti = mt.inverse();
    Mat<dim> mri = mr.inverse();
    Mat<dim> msi = ms.inverse();
    
    ASSERT_EQ( v - t, mti * v );
    ASSERT_EQ( ri * v, mri * v );
    ASSERT_EQ( v / s, msi * v );
    ASSERT_EQ( ( ri * ( v - t ) ) / s, msi * ( mri * ( mti * v ) ) );
    ASSERT_EQ( ( ri * ( v - t ) ) / s, ( msi * mri * mti ) * v );
}

TEST( math_test, mat2 )
{
    Vec2d v( 1, 2 );
    Vec2d t( 5, 6 );
    Rot2 r = Rot2::fromDegrees( 50 );
    Vec2d s( -1, 3 );
    
    mat_test( v, t, r, s );
}

TEST( math_test, mat3 )
{
    Vec3d v( 1, 2, 3 );
    Vec3d t( 5, 6, 7 );
    Rot3 r( Vec3d( -1, -10, 20 ), -2 );
    Vec3d s( -1, 3, -4 );
    
    mat_test( v, t, r, s );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}
