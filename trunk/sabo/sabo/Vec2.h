#ifndef SABO_VEC2_H
#define SABO_VEC2_H

namespace sabo
{

  class Vec2
  {
    public:
      double x, y;
      
      Vec2();
      Vec2( double x, double y );
      
      bool operator==( const Vec2& v ) const;
      
      Vec2 operator-() const;
      Vec2 operator+( const Vec2& v ) const;
      Vec2 operator-( const Vec2& v ) const;
      Vec2 operator*( double d ) const;
      Vec2 operator/( double d ) const;
      Vec2 operator*( const Vec2& v ) const;
      Vec2 operator/( const Vec2& v ) const;
      void operator+=( const Vec2& v );
      void operator-=( const Vec2& v );
      void operator*=( const Vec2& v );
      void operator/=( const Vec2& v );
      void operator*=( double d );
      void operator/=( double d );
      
      double dot( const Vec2& v ) const;
      double squaredLength() const;
      double length() const;
      Vec2 norm() const;
      Vec2 project( const Vec2& v ) const;
      double cross( const Vec2& v ) const;
      Vec2 perp() const;
  };

}

#endif
