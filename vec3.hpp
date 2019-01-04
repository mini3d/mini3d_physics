// Copyright (c) <2012> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>


#ifndef MINI3D_MATH_VEC3_H
#define MINI3D_MATH_VEC3_H

// TODO: Near equals

#include <cmath>
#include <cfloat>

namespace mini3d {
namespace math {

#define MIN(a,b) (a) < (b) ? a : b
#define MAX(a,b) (a) > (b) ? a : b

////////// VECTOR 3 ////////////////////////////////////////////////////////////

class Vec3 {
public:
    union { struct { float x, y, z; }; float xyz[3]; };

    inline Vec3()                           : x(0), y(0), z(0)          {}
    inline Vec3(float x, float y, float z)  : x(x), y(y), z(z)          {}
    inline Vec3(float s)                    : Vec3(s, s, s)             {}
    inline Vec3(const float v[3])           : Vec3(v[0], v[1], v[2])    {}
    inline Vec3(float v[3])                 : Vec3(v[0], v[1], v[2])    {}
    inline Vec3(const Vec3 &v)              : Vec3(v.x, v.y, v.z)       {}

    inline operator float*()                                            { return &x; }
    inline operator const float*() const                                { return &x; }

    inline const Vec3& operator = (const Vec3 &v)                       { x = v.x; y = v.y; z = v.z; return *this; }
    inline const Vec3& operator = (float s)                             { x = s; y = s; z = s; return *this; }

    inline const bool operator == ( const Vec3 &v ) const               { return (v.x==x && v.y==y && v.z == z); }
    inline const bool operator != ( const Vec3 &v ) const               { return !(*this == v); }

    inline const Vec3 operator -() const                                { return Vec3(-x, -y, -z); }
    inline float operator [](int index) const                           { return xyz[index]; }

    // creates all the +, -, *, /, +=, -=, *=, /= operator overloads
    #define OPERATOR(O) \
    inline const Vec3 operator O(const Vec3 &v) const                   { return Vec3(x O v.x, y O v.y, z O v.z); } \
    inline const Vec3 operator O(float s) const                         { return Vec3(x O s, y O s, z O s); } \
    inline const Vec3& operator O ## =(const Vec3 &v)                   { x O ## = v.x, y O ## = v.y, z O ## = v.z; return *this; } \
    inline const Vec3& operator O ## =(float s)                         { x O ## = s, y O ## = s, z O ## = s; return *this; }
    OPERATOR(+); OPERATOR(-); OPERATOR(*); OPERATOR(/);

    inline float Dot(const Vec3 &v) const                               { return x * v.x + y * v.y + z * v.z; }
    inline Vec3 Cross(const Vec3 &v) const                              { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }

    inline void Negate()                                                { x = -x; y = -y; z = -z; }
    inline float Norm() const                                           { return x*x + y*y + z*z; }
    inline float Length() const                                         { return sqrt(Norm()); }
    inline float RectilinearDistance() const                            { return abs(x) + abs(y) + abs(z); }
    inline float Distance(const Vec3 &v) const                          { return (*this - v).Length(); }
    inline float SquareDistance(const Vec3 &v) const                    { return (*this - v).Norm(); }
    inline Vec3& Normalize()                                            { *this /= Length(); return *this; }
    inline const Vec3 Normalized() const                                { return *this / Length(); }
    inline Vec3 Dir(const Vec3 &dir) const                              { return this->Dot(dir) < 0 ? -*this : *this; }

    static Vec3 min(Vec3 a, Vec3 b)                                     { return Vec3(MIN(a.x, b.x), MIN(a.y, b.y), MIN(a.z, b.z)); }
    static Vec3 max(Vec3 a, Vec3 b)                                     { return Vec3(MAX(a.x, b.x), MAX(a.y, b.y), MAX(a.z, b.z)); }
    
    static Vec3 Interpolated(const Vec3 &a, const Vec3 &b, float wA, float wB)                  { Vec3 r; Interpolate(r, a, b, wA, wB); return r; }
    static void Interpolate(Vec3 &result, const Vec3 &a, const Vec3 &b, float wA, float wB)     { result = a + (b - a) * wA / (wA + wB); }

};

#define OPERATOR2(O)\
inline const Vec3 operator O(float s, const Vec3 &v)                    { return v O s; }
OPERATOR2(+); OPERATOR2(-); OPERATOR2(*); OPERATOR2(/);

static const Vec3 UP = Vec3(0,0,1);
static const Vec3 DOWN = Vec3(0,0,-1);
static const Vec3 RIGHT = Vec3(1,0,0);
static const Vec3 LEFT = Vec3(-1,0,0);
static const Vec3 FORWARD = Vec3(0,1,0);
static const Vec3 BACK = Vec3(0,-1,0);

}
}

#endif
