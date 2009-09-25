#ifndef MAT3_H
#define MAT3_H

#include <vector>
#include "fheMath.h"

namespace fhe
{
    
    class Rot;
    class Vec2;
    class Vec3;
    
    typedef std::vector<Vec2> Vec2List;
    
    class Mat3
    {
        private:
            float f[9];
            
        public:
            Mat3();
            Mat3(float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7, float f8);
            Mat3(float* _f);
            Mat3(const Mat3& m);
            
            Mat3& operator=(const Mat3& m);
            
            float get(int i) const;
            float get(int i, int j) const;
            void set(int i, float val);
            void set(int i, int j, float val);
            
            Mat3 operator*(const Mat3& m) const;
            Vec2 operator*(const Vec2& v) const;
            Vec3 operator*(const Vec3& v) const;
            
            void getBasis(Vec3& xAxis, Vec3& yAxis, Vec3& zAxis) const;
            
            Vec2List transform(const Vec2List& vl) const;
            
            float det() const;
            Mat3 inverse() const;
            
            bool equals(const Mat3& m, float eps = Math::EPSILON) const;
    
            static Mat3 translation(const Vec2& v);
            static Mat3 scale(const Vec2& v);
            static Mat3 rotation(const Rot& r);
            
            const static Mat3 ZERO;
            const static Mat3 IDENTITY;
    };
    
}

#endif
