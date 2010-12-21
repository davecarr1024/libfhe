#include <fhe/core/Vec.h>
#include <fhe/core/Rot.h>
#include <fhe/core/Util.h>
#include <sstream>

namespace fhe
{
    template <>
    const Vec2d Vec2d::ZERO( 0, 0 );
    template <>
    const Vec2d Vec2d::UNIT_X( 1, 0 );
    template <>
    const Vec2d Vec2d::UNIT_Y( 0, 1 );
    
    template <>
    std::string Vec2d::typeName()
    {
        return "Vec2d";
    }
    
    template <>
    const Vec3d Vec3d::ZERO( 0, 0, 0 );
    template <>
    const Vec3d Vec3d::UNIT_X( 1, 0, 0 );
    template <>
    const Vec3d Vec3d::UNIT_Y( 0, 1, 0 );
    template <>
    const Vec3d Vec3d::UNIT_Z( 0, 0, 1 );
    
    template <>
    std::string Vec3d::typeName()
    {
        return "Vec3d";
    }
    
    template <>
    const Vec2i Vec2i::ZERO( 0, 0 );
    template <>
    const Vec2i Vec2i::UNIT_X( 1, 0 );
    template <>
    const Vec2i Vec2i::UNIT_Y( 0, 1 );
    
    template <>
    std::string Vec2i::typeName()
    {
        return "Vec2i";
    }
    
    template <>
    const Vec3i Vec3i::ZERO( 0, 0, 0 );
    template <>
    const Vec3i Vec3i::UNIT_X( 1, 0, 0 );
    template <>
    const Vec3i Vec3i::UNIT_Y( 0, 1, 0 );
    template <>
    const Vec3i Vec3i::UNIT_Z( 0, 0, 1 );
    
    template <>
    std::string Vec3i::typeName()
    {
        return "Vec3i";
    }
    
}
