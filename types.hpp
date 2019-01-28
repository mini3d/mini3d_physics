// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#ifndef MINI3D_PHYSICS_TYPES_H
#define MINI3D_PHYSICS_TYPES_H

#include "transform.hpp"
#include "geometry.hpp"

#include <vector>
#include <unordered_set>
#include <list>
#include <cfloat>

namespace mini3d {
namespace physics {

using namespace std;
using namespace mini3d::math;

struct Hull {
    vector<Vec3> vertices;
    vector<Face> faces;
    vector<vector<uint8_t>> faceVertices;
    vector<vector<Vec3>> faceEdgeNormals;
    
    inline int getSupport(const Vec3& normal) const {
        float maxDistance = -FLT_MAX;
        int maxIndex = -1;
        for (int i = 0; i < vertices.size(); ++i) {
            float distance = vertices[i].Dot(normal);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxIndex = i;
            }
        }
        return maxIndex;
    }
};

struct BoxHull : Hull {
    BoxHull(const Vec3 &size) {
        float x = size.x / 2.0f, y = size.y / 2.0f, z = size.z / 2.0f;
        vertices = { Vec3(x,y,z), Vec3(-x,y,z), Vec3(x,-y,z), Vec3(-x,-y,z), Vec3(x,y,-z), Vec3(-x,y,-z), Vec3(x,-y,-z), Vec3(-x,-y,-z) };
        faces = { face(RIGHT, x), { face(FORWARD, y) }, { face(UP, z) }, { face(LEFT, x) }, { face(BACK, y) }, { face(DOWN, z) } };
        faceVertices = { {0, 2, 6, 4}, {0, 1, 5, 4}, {0, 1, 3, 2}, {1, 3, 7, 5}, {2, 3, 7, 6}, {4, 5, 7, 6} };
        faceEdgeNormals = { { UP, BACK, DOWN, FORWARD }, { UP, LEFT, DOWN, RIGHT }, { FORWARD, LEFT, BACK, RIGHT },
                            { UP, BACK, DOWN, FORWARD }, { UP, LEFT, DOWN, RIGHT }, { FORWARD, LEFT, BACK, RIGHT } };
    }

    inline Face face(const Vec3 &dir, float offset)                                    { return { dir * offset, dir }; }
};

struct Manifold;

struct Body {
    Body* next;
    Body* prev;

    inline bool isFixed() const                                                 { return invInertia.x == 0 && invInertia.y == 0 && invInertia.z == 0 && invMass == 0; }
    inline float getRadius(float timeStep) const                                { return radius + velocity.RectilinearDistance() * timeStep; }

    Transform transform = Transform::Identity();
    Vec3 velocity = 0.0f;
    Vec3 angularVelocity = 0.0f;

    Vec3 invInertia = 0.0f;
    float invMass = 0.0f;
    
    float friction = 0.4f;
    float restitution = 0.5f;

    float radius = 0.0f;
    float sleepTimer = 0.0f;
    bool isAwake = true;
    bool isPartOfIsland = false;
    
    size_t shockLevel = 0;
    
    Hull hull;
    
    list<Body*> proxmimities;
    list<Manifold*> manifolds;

    inline Vec3 getVelocity(const Vec3 &point) { return velocity + angularVelocity.Cross(point); }
};

struct Box : Body {

    Box(const Vec3 size) {
        float density = 1.0f;
        float mass = size.x * size.y * size.z * density;
        float x2 = size.x * size.x, y2 = size.y * size.y, z2 = size.z * size.z;

        radius = size.Length() / 2.0f;
        invMass = 1.0f / mass;
        invInertia.x = 1.0f / (1.0f / 12.0f * mass * (y2 + z2));
        invInertia.y = 1.0f / (1.0f / 12.0f * mass * (x2 + z2));
        invInertia.z = 1.0f / (1.0f / 12.0f * mass * (x2 + y2));
        hull = BoxHull(size);
    }
};

}
}

#endif
