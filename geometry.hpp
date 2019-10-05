// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#ifndef MINI3D_MATH_GEOMETRY_H
#define MINI3D_MATH_GEOMETRY_H

#include "transform.hpp"
#include "vec3.hpp"

namespace mini3d {
namespace math {


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RAY
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


struct Ray { Vec3 origin; Vec3 direction; };


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// FACE
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Face {
    Vec3 pos; Vec3 normal;

    Vec3 projection(const Vec3 &point) const                { return point - distance(point) * normal; }
    Vec3 distance(const Vec3 &point) const                  { return (point - pos).Dot(normal); }
    Vec3 intersection(const Ray &ray) const                 { return (pos - ray.origin).Dot(normal) / (ray.direction.Dot(normal)) * ray.direction + ray.origin; }
};


struct Geometry {

static bool raySphereIntersection(const Ray &ray, const Vec3 sphereCenter, const float sphereRadius) {
    Vec3 bodyToLine = (ray.origin - sphereCenter);
    float bodyToLineProjection = ray.direction.Dot(bodyToLine);
    float imag = bodyToLineProjection * bodyToLineProjection + sphereRadius * sphereRadius - bodyToLine.Norm();

    return imag > 0;
}

static inline float rayPlaneIntersectionDistance(const Ray &ray, const Face &plane) {
    float rayDotPlane = ray.direction.Dot(plane.normal);
    return (abs(rayDotPlane) < 0.00001f) ? FLT_MAX : (plane.pos - ray.origin).Dot(plane.normal) / rayDotPlane;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ORTHO NORMAL BASIS VECTORS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// From Erin Catto http://box2d.org/2014/02/computing-a-basis/
// Use normal CROSS tangent to get the binormal
static inline Vec3 getTangent(const Vec3& normal) {
    return ((abs(normal.x) >= 0.57735f) ?
            Vec3(normal.y, -normal.x, 0.0f) :
            Vec3(0.0f, normal.z, -normal.y)).Normalized();
}
};

inline Face operator *(const Transform& t, const Face& f) {
    return { t * f.pos, t.rotate(f.normal) };
}
inline Ray operator *(const Transform& t, const Ray& r) {
    return { t * r.origin, t.rotate(r.direction) };
}

}
}

#endif /* MINI3D_MATH_GEOMETRY_H */
