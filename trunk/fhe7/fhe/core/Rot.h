#ifndef FHE_ROT_H
#define FHE_ROT_H

#include <fhe/core/fhe_math.h>
#include <fhe/core/PyEnv.h>
#include <fhe/core/Util.h>

namespace fhe
{
    
    template <typename T>
    class Rot<2,T>
    {
        private:
            T m_a;
            
            Rot( T a ) :
                m_a( a )
            {
            }
            
        public:
            typedef Vec<2,T> V;
            
            Rot() :
                m_a( 0 )
            {
            }
            
            explicit Rot( const V& v ) :
                m_a( Math::atan2( v.y, v.x ) )
            {
            }
            
            static Rot fromDegrees( T degrees )
            {
                return Rot( Math::radians( degrees ) );
            }
            
            static Rot fromRadians( T radians )
            {
                return Rot( radians );
            }
            
            T degrees() const
            {
                return Math::degrees( m_a );
            }
            
            T radians() const
            {
                return m_a;
            }
            
            Rot operator+( const Rot& r ) const
            {
                return Rot( m_a + r.m_a );
            }
            
            Rot operator-( const Rot& r ) const
            {
                return Rot( m_a - r.m_a );
            }
            
            Rot operator*( T d ) const
            {
                return Rot( m_a * d );
            }
            
            Rot operator/( T d ) const
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                return Rot( m_a / d );
            }
            
            V operator*( const V& v ) const
            {
                return V( Rot( v ) + *this ) * v.length();
            }
            
            bool operator==( const Rot& r ) const
            {
                return equals( r );
            }

            Rot inverse() const
            {
                return Rot( -m_a );
            }
            
            Rot norm() const
            {
                T d = m_a;
                while ( d < -Math::PI )
                {
                    d += Math::PI * 2;
                }
                while ( d > Math::PI )
                {
                    d -= Math::PI * 2;
                }
                return Rot( d );
            }
            
            bool equals( const Rot& r, T eps = Math::EPS ) const
            {
                return Math::equal( m_a, r.m_a, eps );
            }
            
            static std::string typeName()
            {
                return std::string( "Rot2" ) + typeid(T).name();
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                os << typeName() << "(" << m_a << ")";
                return os.str();
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Rot>( typeName().c_str(), boost::python::init<>() )
                    .def( "fromDegrees", &Rot::fromDegrees )
                    .staticmethod( "fromDegrees" )
                    .def( "fromRadians", &Rot::fromRadians )
                    .staticmethod( "fromRadians" )
                    .def( "__eq__", &Rot::operator== )
                    .def( "__repr__", &Rot::toString )
                    .def( boost::python::self + boost::python::other<Rot>() )
                    .def( boost::python::self - boost::python::other<Rot>() )
                    .def( boost::python::self * T() )
                    .def( boost::python::self / T() )
                    .def( "inverse", &Rot::inverse )
                    .def( "norm", &Rot::norm )
                ;
                c.attr( "IDENTITY" ) = IDENTITY;
                return c;
            }
            
            static const Rot IDENTITY;
    };
    
    template <typename T>
    const Rot<2,T> Rot<2,T>::IDENTITY;
    
    template <typename T>
    class Rot<3,T>
    {
        public:
            typedef Vec<3,T> V;
            
            T w, x, y, z;
            
            Rot() :
                w( 1 ),
                x( 0 ),
                y( 0 ),
                z( 0 )
            {
            }
            
            Rot( T _w, T _x, T _y, T _z ) :
                w( _w ),
                x( _x ),
                y( _y ),
                z( _z )
            {
            }
            
            Rot( const V& axis, T angle )
            {
                V axisNorm = axis.norm();
                T sa = Math::sin( angle / 2 );
                T ca = Math::cos( angle / 2 );
                *this = Rot( ca, axisNorm.x * sa, axisNorm.y * sa, axisNorm.z * sa );
            }
            
            explicit Rot( const V& v )
            {
                *this = V::UNIT_X.getRotTo( v );
            }
            
            void toAxisAngle( V& axis, T& angle ) const
            {
                Rot q = norm();
                T ca = q.w;
                angle = Math::acos(ca) * 2.0;
                T sa = Math::sqrt(1.0 - ca * ca);
                if (Math::abs(sa) < Math::EPS)
                    sa = 1;
                axis = V(q.x / sa, q.y / sa, q.z /sa);
            }
            
            Rot operator*( const Rot& r ) const
            {
                return Rot(w * r.w - x * r.x - y * r.y - z * r.z,
                            w * r.x + x * r.w + y * r.z - z * r.y,
                            w * r.y + y * r.w + z * r.x - x * r.z,
                            w * r.z + z * r.w + x * r.y - y * r.x);
            }
            
            V operator*( const V& v ) const
            {
                Rot r = *this * Rot(0,v.x,v.y,v.z) * inverse();
                return V(r.x,r.y,r.z);
            }
            
            Rot operator*( T d ) const
            {
                return Rot(w * d, x * d, y * d, z * d);
            }
            
            Rot operator/( T d ) const
            {
                FHE_ASSERT( !Math::equal( d, 0 ) );
                return Rot(w / d, x / d, y / d, z / d);
            }
            
            bool operator==( const Rot& r ) const
            {
                return equals( r );
            }
            
            Rot conjugate() const
            {
                return Rot(w,-x,-y,-z);
            }
            
            T magnitude() const
            {
                return Math::sqrt(w * w + x * x + y * y + z * z);
            }
            
            Rot norm() const
            {
                return *this / magnitude();
            }
            
            Rot inverse() const
            {
                return norm().conjugate();
            }
            
            bool equals( const Rot& r, T eps = Math::EPS ) const
            {
                return Math::equal(w,r.w,eps) && 
                       Math::equal(x,r.x,eps) && 
                       Math::equal(y,r.y,eps) && 
                       Math::equal(z,r.z,eps);
            }
            
            static std::string typeName()
            {
                return std::string( "Rot3" ) + typeid(T).name();
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                V axis;
                T angle;
                toAxisAngle( axis, angle );
                os << typeName() << "(" << axis << "," << angle << ")";
                return os.str();
            }

            boost::python::object pyToAxisAngle()
            {
                V axis;
                T angle;
                toAxisAngle(axis,angle);
                return boost::python::make_tuple(axis,angle);
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Rot>(typeName().c_str(),boost::python::init<>())
                    .def(boost::python::init<T,T,T,T>())
                    .def(boost::python::init<V,T>())
                    .def(boost::python::init<V>())
                    .def_readwrite("w", &Rot::w)
                    .def_readwrite("x", &Rot::x)
                    .def_readwrite("y", &Rot::y)
                    .def_readwrite("z", &Rot::z)
                    .def("__repr__", &Rot::toString)
                    .def("__eq__", &Rot::operator== )
                    .def(boost::python::self * boost::python::other<Rot>())
                    .def(boost::python::self * boost::python::other<V>())
                    .def(boost::python::self * T())
                    .def(boost::python::self / T())
                    .def("norm",&Rot::norm)
                    .def("inverse",&Rot::inverse)
                    .def("toAxisAngle",&Rot::pyToAxisAngle)
                ;
                c.attr( "IDENTITY" ) = IDENTITY;
                return c;
            }
            
            static const Rot IDENTITY;
    };
    
    template <typename T>
    const Rot<3,T> Rot<3,T>::IDENTITY;

    template <size_t dim, typename T>
    std::ostream& operator<<( std::ostream& os, const Rot<dim,T>& r )
    {
        return os << r.toString();
    }
    
}

#endif
