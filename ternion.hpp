
// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>


#ifndef MINI3D_MATH_TERNION_H
#define MINI3D_MATH_TERNION_H

#include "vec3.hpp"
#include <cfloat>

namespace mini3d {
namespace math {

////////// TERNION /////////////////////////////////////////////////////////////
// https://www.ma.utexas.edu/mp_arc/c/05/05-53.pdf


class Ternion
{
public:

    union { struct { float x, y, z; }; float xyz[3]; };

    inline Ternion()                                                            : x(0), y(0), z(0) {}
    inline Ternion(float x, float y, float z)                                   : x(x), y(y), z(z) {}
    inline Ternion(float s)                                                     : Ternion(s, s, s) {}
    inline Ternion(const float v[3])                                            : Ternion(v[0], v[1], v[2]) {}
    inline Ternion(float v[3])                                                  : Ternion(v[0], v[1], v[2]) {}
    inline Ternion(const Ternion &v)                                            : Ternion(v.x, v.y, v.z) {}

    inline Ternion(float x, float y, float z, float w)                          : Ternion(x/w, y/w, z/w) {}

    static inline Ternion FromAxisAngle(float x, float y, float z, float a)            { float tanA = tan(a * 0.5f); return Ternion(tanA * x, tanA * y, tanA * z); }
    static inline Ternion FromTwoVectors(const Vec3 &u, const Vec3 &v)                 { Vec3 w = u.Cross(v); return Ternion(w / (1.0f + u.Dot(v) + FLT_MIN)); }
    inline float GetAngle() const                                               { return 2.0f * atanf(sqrt(x * x + y * y + z * z)); }

    inline const Ternion& operator = (const Vec3 &v)                            { x = v.x; y = v.y; z = v.z; return *this; }

    inline const Ternion operator -() const                                     { return Ternion(-x, -y, -z); }

    inline float operator [](int index) const                                   { return xyz[index]; }

    inline const Ternion operator +(const Ternion &q) const                     { return Ternion(x + q.x, y + q.y, z + q.z); }
    inline const Ternion operator *(float s) const                              { return Ternion(x * s, y * s, z * s); }
    inline const Vec3 operator *(const Vec3 &v) const                           { return Vec3((*this * Ternion(v) * -*this).xyz); } // TODO: Optimize?
    inline const Ternion operator *(const Ternion &q) const                     { float s = 1 - x * q.x - y * q.y - z * q.z;
                                                                                  return Ternion(x + q.x + y * q.z - z * q.y,
                                                                                              y + q.y + z * q.x - x * q.z,
                                                                                              z + q.z + x * q.y - y * q.x) *
                                                                                              (abs(s) < 0.001f ? 1.0f / 0.00001f : 1.0f / s); }

    inline const Ternion operator *=(const Ternion &q)                          { return *this = *this * q; }
    inline const Ternion operator +=(const Ternion &q)                          { return *this = *this + q; }

    inline Vec3 Transform(const Vec3 v) const                                   { return *this * v; }
    inline static Vec3 Transform(const Ternion &q, const Vec3 &v)               { return q.Transform(v); }

    inline void RotateAxis(const Ternion &q)                                    { *this = q * *this * -q; }
    inline const Ternion RotatedAxis(const Ternion &q) const                    { return q * *this * -q; }
};

inline const Ternion operator *(float s, const Ternion &q)                      { return q * s; }
inline const Ternion operator /(float s, const Ternion &q)                      { return q * (1 / s); }

}
}

#endif
