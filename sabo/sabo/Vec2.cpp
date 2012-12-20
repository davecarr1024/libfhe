#include <sabo/Vec2.h>
#include <cmath>

namespace sabo
{

  Vec2::Vec2() :
    x( 0 ),
    y( 0 )
  {
  }
  
  Vec2::Vec2( double _x, double _y ) :
    x( _x ),
    y( _y )
  {
  }
  
  bool Vec2::operator==( const Vec2& v ) const
  {
    static const double eps = 1e-5;
    return fabs( x - v.x ) < eps && fabs( y - v.y ) < eps;
  }
  
  Vec2 Vec2::operator-() const
  {
    return Vec2( -x, -y );
  }
  
  Vec2 Vec2::operator+( const Vec2& v ) const
  {
    return Vec2( x + v.x, y + v.y );
  }

  Vec2 Vec2::operator-( const Vec2& v ) const
  {
    return Vec2( x - v.x, y - v.y );
  }

  Vec2 Vec2::operator*( const Vec2& v ) const
  {
    return Vec2( x * v.x, y * v.y );
  }

  Vec2 Vec2::operator/( const Vec2& v ) const
  {
    return Vec2( x / v.x, y / v.y );
  }

  Vec2 Vec2::operator*( double d ) const
  {
    return Vec2( x * d, y * d );
  }

  Vec2 Vec2::operator/( double d ) const
  {
    return Vec2( x / d, y / d );
  }
  
  void Vec2::operator+=( const Vec2& v )
  {
    x += v.x;
    y += v.y;
  }

  void Vec2::operator-=( const Vec2& v )
  {
    x -= v.x;
    y -= v.y;
  }

  void Vec2::operator*=( const Vec2& v )
  {
    x *= v.x;
    y *= v.y;
  }

  void Vec2::operator/=( const Vec2& v )
  {
    x /= v.x;
    y /= v.y;
  }
  
  void Vec2::operator*=( double d )
  {
    x *= d;
    y *= d;
  }
  
  void Vec2::operator/=( double d )
  {
    x /= d;
    y /= d;
  }
  
  double Vec2::dot( const Vec2& v ) const
  {
    return x * v.x + y * v.y;
  }
  
  double Vec2::squaredLength() const
  {
    return x * x + y * y;
  }
  
  double Vec2::length() const
  {
    return sqrt( x * x + y * y );
  }
  
  Vec2 Vec2::norm() const
  {
    return operator/( length() );
  }
  
  Vec2 Vec2::project( const Vec2& v ) const
  {
    return v * ( dot( v ) / v.squaredLength() );
  }
  
  double Vec2::cross( const Vec2& v ) const
  {
    return x * v.y - y * v.x;
  }
  
  Vec2 Vec2::perp() const
  {
    return Vec2( -y, x );
  }

}
