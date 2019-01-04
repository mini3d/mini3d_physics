
// Copyright (c) <2012> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>


#ifndef MINI3D_MATH_TRANSFORM_H
#define MINI3D_MATH_TRANSFORM_H

#include "vec3.hpp"
#include "ternion.hpp"

namespace mini3d {
namespace math {

// Fist scales, then rotates, then translates

////////// TRANSFORM //////////////////////////////////////////////////////////

struct Transform {

    Vec3 pos;
    Ternion rot;
    Vec3 scale;

    Transform()                                                                 : pos(Vec3(0.0f)), rot(Ternion()), scale(1.0f) { };
    Transform(const Vec3 pos, const Ternion rot, const Vec3 scale)              : pos(pos), rot(rot), scale(scale) {}
    Transform(const Transform &t)                                               : pos(t.pos), rot(t.rot), scale(t.scale) { };
    Transform(const float pos[3], const float rot[4], float scale[3])           : pos(pos), rot(rot), scale(scale) {}
    Transform(const Vec3 pos, const Ternion rot)                                : pos(pos), rot(rot), scale(1) {}

    static Transform Identity()                                                 { return Transform(Vec3(0.0f), Ternion(0.0f, 0.0f, 0.0f, 1.0f), 1.0f); }
    static Transform fromTo(const Vec3 p0, const Vec3 t0, const Vec3 p1, const Vec3 t1) { return Transform(p1 - p0, Ternion::FromTwoVectors(t0, t1)); }

    inline Transform operator *(const Transform &t) const                       { return Transform(*this * t.pos, rot * t.rot, scale * t.scale); }
    inline Vec3 operator *(const Vec3 &v) const                                 { return pos + rot.Transform(v * scale); }
    inline Transform operator -() const                                         { return Transform(-(-rot).Transform(pos), -rot); }
    
    inline Vec3 rotate(const Vec3 &v) const                                     { return rot.Transform(v); }
    inline Vec3 rotateInv(const Vec3 &v) const                                  { return (-rot).Transform(v); }
};

}
}

#endif
