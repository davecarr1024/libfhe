#ifndef MAT4_H
#define MAT4_H

#include "Vec3.h"
#include "ggeMath.h"

#include <iostream>

namespace gge
{
    
    class Quat;
    class Mat3;
    
    class Mat4
    {
        private:
            float f[16];
            
        public:
            Mat4();
            Mat4(float* _f);
            Mat4(float f0, float f1, float f2, float f3, 
                 float f4, float f5, float f6, float f7, 
                 float f8, float f9, float f10, float f11, 
                 float f12, float f13, float f14, float f15);
            Mat4(const Mat4& m);
            
            Mat4& operator=(const Mat4& m);
            
            float get(int i) const;
            float get(int i, int j) const;
            void set(int i, float val);
            void set(int i, int j, float val);
            
            Mat4 operator*(const Mat4& m) const;
            Vec3 operator*(const Vec3& v) const;
            
            void transform(Vec3List& vl) const;
            
            Vec3 getTranslation() const;
            Quat getRotation() const;
            Mat3 getRotationMat() const;
            void getBasis(Vec3& xAxis, Vec3& yAxis, Vec3& zAxis) const;
            
            Mat3 submat(int i, int j) const;
            float det() const;
            Mat4 inverse() const;
            
            bool equals(const Mat4& m, float eps = Math::EPSILON) const; 
            
            friend std::ostream& operator<<(std::ostream& os, const Mat4& m);
            
            static Mat4 translation(const Vec3& v);
            static Mat4 scale(const Vec3& v);
            static Mat4 rotation(const Quat& q);
    
            const static Mat4 ZERO;
            const static Mat4 IDENTITY;
    };
    
}

#endif
