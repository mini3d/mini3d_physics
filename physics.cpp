// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#include "physics.hpp"
#include "transform.hpp"
#include "geometry.hpp"

#include <random>
#include <vector>
#include <list>
#include <cfloat>
#include <cmath>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include <string>

// TODO: Remove
#include <GLUT/glut.h>


// TODO: There is some bug with fricition that makes the objects sway, float around an such.
// TODO: It could also be in the inertia calculations because the position solver sometimes makes objects shift over in the tangent direciton
// TODO: If you set inv inertia to 0.0f it goes away (but you get bouncy stacks instead)

namespace mini3d {
namespace physics {

using namespace std;
using namespace mini3d::math;

enum { NORMAL = 0, TANGENT = 1, BINORMAL = 2, NORMALS_COUNT = 3 };
enum { FIRST = 0, SECOND = 1, COLLISION_PARTIES_COUNT = 2 };

bool paused = false;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SUPPORT POINT
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SupportPoint {
    Vec3 p;
    size_t i;
    size_t j;
};

const Vec3 getSupport(const Transform &tfa, const Transform &tfb, const Hull &a, const Hull &b, const size_t i, const size_t j) {
    return tfa * a.vertices[i] - tfb * b.vertices[j];
}

const SupportPoint getSupport(const Transform &tfa, const Transform &tfb, const Hull &a, const Hull &b, const Vec3 &n) {
    Vec3 localNormalA = tfa.rotateInv(n);
    Vec3 localNormalB = tfb.rotateInv(-n);
    size_t i = a.getSupport(localNormalA);
    size_t j = b.getSupport(localNormalB);
    return { getSupport(tfa, tfb, a, b, i, j), i, j };
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SIMPLEX
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A simplex temporarily wraps a cache and a set of support vectors
struct Simplex {
    size_t count;
    array<SupportPoint, 4> p;
    
    const SupportPoint& operator[] (size_t i) const { return p[i]; }
    SupportPoint& operator[] (size_t i) { return p[i]; }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CONTACT
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A clip point is represented by the point cooridnates and the indices of the two edges that clipped the point
// If the point is inside the clipping region then i[1] == UINT8_MAX (representing no clipping edge)
struct ClipPoint { Vec3 p; uint8_t index; uint8_t clipIndex; };

struct Contact {
    ClipPoint clipPoint;
    Vec3 pos[COLLISION_PARTIES_COUNT];
    Vec3 pos0[COLLISION_PARTIES_COUNT];
    Vec3 inertia[COLLISION_PARTIES_COUNT][NORMALS_COUNT];
    
    bool isSleeping;
    float sleepTimer;

    float friction;
    float velocityBias;
    float normalVelocity;

    Vec3 n[NORMALS_COUNT];

    float distance;
    float impulse[NORMALS_COUNT];
    float mass[NORMALS_COUNT];
    float correction;
    
    float isStatic;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MANIFOLD
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Manifold {
    Body* body[COLLISION_PARTIES_COUNT];

    Simplex simplex;
    SupportPoint sps[2];
    int spsCount = 0;

    array<Contact, 8> contacts;
    size_t contactsCount;
    
    Vec3 axis = Vec3(1,0,0);
    
    bool ignoreContacts = false;
    bool contactStartedCalled = false;
    bool isPartOfIsland = false;
    bool isFixed = false;
    
    bool operator==(const Manifold &other) const { return (body[0] == other.body[0] && body[1] == other.body[1]) || (body[1] == other.body[0] && body[0] == other.body[1]); }
};

list<Manifold> manifolds;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// BROAD PHASE
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool testBroadPhaseProximity(const Body* a, const Body* b , const float timeStep) {
    float radius = (a->getRadius(timeStep) + b->getRadius(timeStep)) * 1.2f;
    return (a->transform.pos.SquareDistance(b->transform.pos) <= radius * radius);
}


template <typename T>
void doBroadPhase(list<Body> &bodies, size_t hullCount, float timeStep, T proximity) {
    for (auto itA = bodies.begin(); itA != bodies.end(); ++itA) {
        
        // TODO: Clean up
        auto itB = itA;
        itB++;
        for ( ; itB != bodies.end(); ++itB) {
            if (!(itA->isFixed() && itB->isFixed()) && (itA->isAwake || itB->isAwake) && testBroadPhaseProximity(&(*itA), &(*itB), timeStep)) {
                proximity(&(*itA), &(*itB));
            }
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// COLLISION DETECTION
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Fraction {
    float distance;
    float norm;

    inline bool operator <(const Fraction &o) const {
        return distance * abs(distance) * o.norm < o.distance * abs(o.distance) * norm;
    }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GEOMETRY
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static Vec3 fromLinePerpendicularTowardsOrigo(const Vec3 &l0, const Vec3 &l1) {
    Vec3 line = l0 - l1; return (line).Cross(-l0).Cross(line);
}

static Vec3 faceNormal(const Vec3 &f0, const Vec3 &f1, const Vec3 &f2) {
    return (f1 - f0).Cross(f1 - f2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// EPA
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Edge { size_t a; size_t b; };
struct Triangle { array<size_t, 3> i; Vec3 n; Fraction u; };

Vec3 normal(const SupportPoint &a, const SupportPoint &b, const SupportPoint &c)    { return (b.p - a.p).Cross(c.p - a.p); }
Triangle makeTri(array<size_t, 3> i, const vector<SupportPoint> &p)                 { Vec3 n = normal(p[i[0]], p[i[1]], p[i[2]]); return { i, n, { n.Dot(p[i[0]].p), n.Norm() }}; };

// Based on Jacob Tyndalls EPA implementation for Lattice3D (https://bitbucket.org/Hacktank/lattice3d)
void doEPA(const Transform &tfa, const Transform &tfb, const Hull &hull, const Hull &hull2, Simplex &s) {

    // Make sure winding of simplex is correct
    if (normal(s[0], s[1], s[2]).Dot(s[0].p) < 0) swap(s[0], s[1]);

    vector<SupportPoint> points = { s[0], s[1], s[2], s[3] };
    vector<Triangle> triangles = { makeTri({0, 1, 2}, points), makeTri({0, 2, 3}, points), makeTri({0, 3, 1}, points), makeTri({1, 3, 2}, points) };

    vector<Edge> edges;
    auto markEdge = [&](const Edge &e)->void {
        for(auto &edge : edges) {
            if(edge.a == e.b && edge.b == e.a) {
                edge = edges.back();
                edges.pop_back();
                return;
            }
        }
        edges.push_back(e);
    };

    int iterations = 0;
    for ( ;; ) {

        auto triangle = max_element(triangles.begin(), triangles.end(),
                [](const Triangle &a, const Triangle &b) { return b.u < a.u; });

        const SupportPoint sp = getSupport(tfa, tfb, hull, hull2, triangle->n);

        // Distance to closest
        if(((triangle->n.Dot(sp.p)) - triangle->u.distance <= triangle->u.norm * Physics::COLLISION_EPSILON)) {
            Vec3 p0 = points[triangle->i[0]].p;
            Vec3 p1 = points[triangle->i[1]].p;
            Vec3 p2 = points[triangle->i[2]].p;

            s =  { 3, points[triangle->i[0]], points[triangle->i[1]], points[triangle->i[2]] };
            return;
        }

        // Check that the new axis defines a plane that separates the new support point from the old ones by some margin
        // Makes sure that:
        // 1. The new support point is better than all of the old ones
        // 2. The new search axis is straigt
        // 3. The new simplex is large enough and affinely independent (not malformed)
        for (int j = 0; j < s.count; ++j) {
            float d = (sp.p - s[j].p).Dot(triangle->n);
            if (d * d < Physics::COLLISION_EPSILON_SQUARED * triangle->n.Norm()) {

                Vec3 p0 = points[triangle->i[0]].p;
                Vec3 p1 = points[triangle->i[1]].p;
                Vec3 p2 = points[triangle->i[2]].p;

                s =  { 3, points[triangle->i[0]], points[triangle->i[1]], points[triangle->i[2]] };

                return;
            }
        }

        // TODO: This sometimes fails
        if (iterations++ > 30) {
            assert(false);
        }

        for(auto it = triangles.begin(); it != triangles.end();) {
            if((it->n.Dot(sp.p) - it->u.distance) > 0) {
                markEdge({ it->i[0], it->i[1] });
                markEdge({ it->i[1], it->i[2] });
                markEdge({ it->i[2], it->i[0] });
                *it = triangles.back();
                triangles.pop_back();
                continue;
            }
            it++;
        }

        points.push_back(sp);
        for(auto &edge : edges) {
            triangles.push_back(makeTri({ points.size() - 1, edge.a, edge.b }, points));
        }

        edges.clear();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// STOCHASTIC SEPARATING AXIS SEARCH
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t state = 98127308;
int32_t xorshift32() {
    int32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
}

// This is a last resort when EPA fails
SupportPoint findSeparatingAxis(const Transform &tfa, const Transform &tfb, const Hull &hull, const Hull &hull2, Vec3 &bestAxis, SupportPoint sps[2], int &spsCount, Manifold &manifold) {
    // assert(false); // TODO: We dont want to end up here
    
    //TODO: Change this to just sample a fixed uniform set of direcitons instead!
    
    SupportPoint bestSupportPoint = getSupport(tfa, tfb, hull, hull2, bestAxis);
    Fraction bestDistance = { -bestSupportPoint.p.Dot(bestAxis), bestAxis.Norm() };

    Vec3 outwards = (tfb.pos - tfa.pos);

/*
    Vec3 defaultAxis = (tfb.pos - tfa.pos).Norm() < Physics::COLLISION_EPSILON_SQUARED ? tfb.pos - tfa.pos : Vec3(0,1,0);
    SupportPoint defaultSupportPoint = getSupport(tfa, tfb, hull, hull2, defaultAxis);
    Fraction defaultDistance = { -defaultSupportPoint.p.Dot(defaultAxis), defaultAxis.Norm() };

    bestAxis = defaultDistance < bestDistance ? bestAxis : defaultAxis;
    bestSupportPoint = defaultDistance < bestDistance ? bestSupportPoint : defaultSupportPoint;
    bestDistance = defaultDistance < bestDistance ? bestDistance : defaultDistance;
*/

    size_t iterations = bestAxis == Vec3(2,0,0) ? 50 : 10;
    
    // Search random directions.
    size_t total = 0;
    for (int i = 0; i < iterations; ++i) {

        // Randomly try a different direction in the vicinity of bestDir
        // delta * delta * delta gives a distribution that is denser around bestDir
        Vec3 delta(xorshift32() / (float)INT32_MAX, xorshift32() / (float)INT32_MAX, xorshift32() / (float)INT32_MAX);
        Vec3 newAxis = (bestAxis + 5.0f * delta * delta * delta * delta * delta).Normalized();

        total++;
        SupportPoint newSupportPoint = getSupport(tfa, tfb, hull, hull2, newAxis);
        Fraction newDistance = { -newSupportPoint.p.Dot(newAxis), newAxis.Norm() };

        if (bestDistance < newDistance) {
            bestAxis = newAxis;
            bestDistance = newDistance;
            bestSupportPoint = newSupportPoint;
            i = 0;
        }
/*
        int path = 0;
        if (spsCount == 0) {
            sps[0] = newSupportPoint;
            spsCount++;
            continue;
        } else if (spsCount == 1) {
            if (sps[0].p == newSupportPoint.p) {
                newAxis = -sps[0].p.Dir(-outwards).Normalized();
                spsCount = 0;
                path = 1;
            } else {
                sps[1] = newSupportPoint;
                spsCount++;
                continue;
            }
        } else if (spsCount == 2) {
            if (sps[0].p == newSupportPoint.p || sps[1].p == newSupportPoint.p) {
                newAxis = (sps[1].p - sps[0].p).Cross(-sps[1].p).Cross(sps[1].p - sps[0].p).Dir(outwards).Normalized();
                spsCount = 0;
                path = 2;
            } else {
                newAxis = ((sps[1].p - sps[0].p).Cross(newSupportPoint.p - sps[0].p)).Dir(outwards).Normalized();
                spsCount = 0;
                path = 3;
            }
        }

        total++;
        newSupportPoint = getSupport(tfa, tfb, hull, hull2, newAxis);
        newDistance = { -newSupportPoint.p.Dot(newAxis), newAxis.Norm() };

        if (bestDistance < newDistance) {
            bestAxis = newAxis;
            bestDistance = newDistance;
            bestSupportPoint = newSupportPoint;
            i = 0;
        }
 */

    }
    //printf("Total: %u\n", total);
    return bestSupportPoint;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GJK
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Test if origo is in the region of v0
inline bool testVertexTetrahedron(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3) {
    return
        -v0.Dot(v1 - v0) < 0 &&
        -v0.Dot(v2 - v0) < 0 &&
        -v0.Dot(v3 - v0) < 0;
}

// Test if origo is in the region of edge (v0, v1) (after testing vertices)
inline bool testEdgeTetrahedron(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3) {

    const Vec3 n012 = (v1 - v0).Cross(v2 - v0);
    const Vec3 n031 = (v3 - v0).Cross(v1 - v0);

    return
        -v0.Dot(v1 - v0) >= 0 &&
        -v1.Dot(v0 - v1) >= 0 &&
        -v0.Dot((v1 - v0).Cross(n012)) > 0 &&
        -v0.Dot(n031.Cross(v1 - v0)) > 0;
}

// Test if origo is in the region of the face 012
inline bool testTriangleTetrahedron(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, const Vec3 &v3) {
    const Vec3 n012 = (v1 - v0).Cross(v2 - v0);

    return
        -v0.Dot((v1 - v0).Cross(n012)) <= 0 &&
        -v1.Dot((v2 - v1).Cross(n012)) <= 0 &&
        -v2.Dot((v0 - v2).Cross(n012)) <= 0 &&
        -v0.Dot(n012) * (v3 - v0).Dot(n012) < 0;
}

// Test if origo is in the region of a vertex on a triangle
inline bool testVertexTriangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) {
    return
        -v0.Dot(v1 - v0) < 0 &&
        -v0.Dot(v2 - v0) < 0;
}

// Test if origo is in the region of an edge on a triangle (after testing vertices)
inline bool testEdgeTriangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) {
    const Vec3 n012 = (v1 - v0).Cross(v2 - v0);

    return
        -v0.Dot(v1 - v0) >= 0 &&
        -v1.Dot(v0 - v1) >= 0 &&
        -v0.Dot((v1 - v0).Cross(n012)) > 0;
}

// Test if origo is in the region of a vertex on a line
inline bool testVertexEdge(const Vec3 &v0, const Vec3 &v1) {
    return
        -v0.Dot(v1 - v0) < 0;
}

int count4 = 0;
int count3 = 0;
int count2 = 0;

int steps[20] = {0};

void doGJK(const Transform &tfa, const Transform &tfb, const Hull &hull, const Hull &hull2, Simplex &s, Vec3 &axis, int iterations, Manifold &manifold) {
    float minDistance = FLT_MAX;
    
    if (s.count != 0) {
        // Update support point world space coordinates
        for (size_t i = 0; i < s.count; ++i) {
            s[i].p = getSupport(tfa, tfb, hull, hull2, s[i].i, s[i].j);
        }
        
        // Check if simplex has become degenerate
        switch (s.count) {
            case 2: if ((s[1].p - s[0].p).Norm() < Physics::COLLISION_EPSILON_SQUARED) s.count = 0; break;
            // TODO: EPA seems to be returning degenerate 3-simplices somtimes (lines)
            case 3: if (faceNormal(s[0].p, s[1].p, s[2].p).Norm()< Physics::COLLISION_EPSILON_SQUARED) s.count = 0; break;
        }
    }

    if (s.count == 0) {
        // Initialize new simplex
        axis = (tfb.pos - tfa.pos).Norm() < Physics::COLLISION_EPSILON_SQUARED ? tfb.pos - tfa.pos : Vec3(0,0,1);
        s = { 1, getSupport(tfa, tfb, hull, hull2, axis) };
    }
    
    size_t i = 0;
    for ( ;; i++) {

        // For each simplex level, keep the simplex if origo is inside, otherwise keep the subsimplex containing the closest point.
        switch (s.count) {
            case 4: {
                count4++;
                if (testVertexTetrahedron(s[0].p, s[1].p, s[2].p, s[3].p)) { s = {1, { s[0] }}; break; }
                if (testVertexTetrahedron(s[1].p, s[2].p, s[3].p, s[0].p)) { s = {1, { s[1] }}; break; }
                if (testVertexTetrahedron(s[2].p, s[3].p, s[0].p, s[1].p)) { s = {1, { s[2] }}; break; }
                if (testVertexTetrahedron(s[3].p, s[0].p, s[1].p, s[2].p)) { s = {1, { s[3] }}; break; }

                if (testEdgeTetrahedron(s[0].p, s[1].p, s[2].p, s[3].p)) { s = {2, { s[0], s[1] }}; break; }
                if (testEdgeTetrahedron(s[0].p, s[2].p, s[1].p, s[3].p)) { s = {2, { s[0], s[2] }}; break; }
                if (testEdgeTetrahedron(s[0].p, s[3].p, s[1].p, s[2].p)) { s = {2, { s[0], s[3] }}; break; }
                if (testEdgeTetrahedron(s[1].p, s[2].p, s[0].p, s[3].p)) { s = {2, { s[1], s[2] }}; break; }
                if (testEdgeTetrahedron(s[1].p, s[3].p, s[0].p, s[2].p)) { s = {2, { s[1], s[3] }}; break; }
                if (testEdgeTetrahedron(s[2].p, s[3].p, s[0].p, s[1].p)) { s = {2, { s[2], s[3] }}; break; }

                if (testTriangleTetrahedron(s[0].p, s[1].p, s[2].p, s[3].p)) { s = {3, { s[0], s[1], s[2] }}; break; }
                if (testTriangleTetrahedron(s[1].p, s[2].p, s[3].p, s[0].p)) { s = {3, { s[1], s[2], s[3] }}; break; }
                if (testTriangleTetrahedron(s[2].p, s[3].p, s[0].p, s[1].p)) { s = {3, { s[2], s[3], s[0] }}; break; }
                if (testTriangleTetrahedron(s[3].p, s[0].p, s[1].p, s[2].p)) { s = {3, { s[3], s[0], s[1] }}; break; }

                // inside tetrahedron
                return;

            } case 3: {
                count3++;

                if (testVertexTriangle(s[0].p, s[1].p, s[2].p)) { s = {1, { s[0] }}; break; }
                if (testVertexTriangle(s[1].p, s[2].p, s[0].p)) { s = {1, { s[1] }}; break; }
                if (testVertexTriangle(s[2].p, s[0].p, s[1].p)) { s = {1, { s[2] }}; break; }

                if (testEdgeTriangle(s[0].p, s[1].p, s[2].p)) { s = {2, { s[0], s[1] }}; break; }
                if (testEdgeTriangle(s[1].p, s[2].p, s[0].p)) { s = {2, { s[1], s[2] }}; break; }
                if (testEdgeTriangle(s[2].p, s[0].p, s[1].p)) { s = {2, { s[2], s[0] }}; break; }

                // inside triangle
                break;
                
            } case 2: {
                count2++;
                if (testVertexEdge(s[0].p, s[1].p)) { s = {1, { s[0] }}; break; }
                if (testVertexEdge(s[1].p, s[0].p)) { s = {1, { s[1] }}; break; }
                
                // inside edge
                break;
            }
        }

        switch(s.count) {
            case 1: axis = -s[0].p; break;
            case 2: axis = fromLinePerpendicularTowardsOrigo(s[0].p, s[1].p); break;
            case 3: axis = faceNormal(s[0].p, s[1].p, s[2].p).Dir(-s[0].p); break;
        }

        float distanceDotAxis = axis.Dot(-s[0].p);
        float distanceToSimplex = distanceDotAxis * distanceDotAxis / axis.Norm();
        
        // Distance to closest point on simplex must be strictly decreasing unless we have found the best simplex
        // Make sure we make decent progress each step. Otherwise we are probably stuck in a loop.
        if (minDistance < distanceToSimplex) {
            steps[i]++;
            return;
        } else {
            minDistance = distanceToSimplex;
        }

        // Check that the new axis defines a plane that separates the new support point from the old ones by some margin
        // Makes sure that:
        // 1. The new support point is better than all of the old ones
        // 2. The new search axis is straigt
        // 3. The new simplex is large enough and affinely independent (not malformed)
        SupportPoint sp = getSupport(tfa, tfb, hull, hull2, axis);
        for (int j = 0; j < s.count; ++j) {
            float d = (sp.p - s[j].p).Dot(axis);
            if (d * d < Physics::COLLISION_EPSILON_SQUARED * axis.Norm()) {
                steps[i]++;
                return;
            }
        }

        // Last resort, if we have started looping, return the current best simplex.
        if (i > 20) {
            assert(false);
        }

        // TODO: Sometimes there is looping when origo is on the boundary of the simplex (axis is a 0-vector)
        // TODO: Maybe the dot tests in the subroutine are not consistent for the boundary

        s[s.count++] = sp;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CONTACTS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t replace = 0; // TODO: remove

void generateClipPointsFromFace(vector<ClipPoint> &result, const Body &body, const Face &clippingPlane, const size_t index) {
    const Transform &transform = body.transform;
    const Hull &hull = body.hull;

    uint8_t i = 0;
    for(auto vertexIndex : hull.faceVertices[index]) {
        Vec3 point = clippingPlane.projection(transform * hull.vertices[vertexIndex]);
        result.push_back({ point, i++, UINT8_MAX });
    };
}

vector<ClipPoint> temp;

// Clip the incident polygon agains the reference polygon
void polygonPolygonClipping(vector<ClipPoint> &subject, vector<ClipPoint> &reference, const Face &clipPlane) {
    float sign = std::copysign(1, (reference[2].p - reference[1].p).Cross(reference[0].p - reference[1].p).Dot(clipPlane.normal));

    uint8_t clipIndex = 0;
    ClipPoint c1 = reference.back();
    for (const auto &c2 : reference ) {
        Vec3 clipEdgeNormal = sign * (c2.p - c1.p).Cross(clipPlane.normal);

        temp.clear();

        const ClipPoint* start = &subject.back();
        float dStart = (start->p - c1.p).Dot(clipEdgeNormal);
        
        for(const auto &end : subject) {
            float dEnd = (end.p - c1.p).Dot(clipEdgeNormal);

            // If start and end point are on different sides of the clipping edge,
            // add the clipping plane intersection point to the output
            if (dStart * dEnd < 0) {
                Vec3 p = Vec3::Interpolated(start->p, end.p, abs(dStart), abs(dEnd));
                temp.push_back({ p, start->index, clipIndex });
            }
        
            // If end point is inside polygon, add it to the output
            if (dEnd <= 0) {
                temp.push_back(end);
            }
            
            start = &end;
            dStart = dEnd;
        }
        subject = temp;
        c1 = c2;
    }
}

void reduceContacts(const Manifold* manifold, const Vec3 &normal, const Face &faceA, const Face &faceB, vector<ClipPoint> &clipPoints, vector<ClipPoint> &clipPointsA) {

    clipPoints.clear();

    // TODO: Make pointers instead
    Body &a = *manifold->body[FIRST];
    Body &b = *manifold->body[SECOND];

    // Get depest point
    if (clipPointsA.size() > 0) {
        ClipPoint* best = nullptr;
        float maxDistance = FLT_MAX;
        for (auto & cp : clipPointsA) {
            Vec3 posA = cp.p - (cp.p - a.transform * faceA.pos).Dot(a.transform.rotate(faceA.normal)) * a.transform.rotate(faceA.normal) + Physics::COLLISION_OFFSET * normal;
            Vec3 posB = cp.p - (cp.p - b.transform * faceB.pos).Dot(b.transform.rotate(faceB.normal)) * b.transform.rotate(faceB.normal) - Physics::COLLISION_OFFSET * normal;

            float distance = -(posB - posA).Dot(normal);
            if (distance < maxDistance) {
                maxDistance = distance;
                best = &cp;
            }
        }
        clipPoints.push_back(*best);
    }

    // Get point furthest from first point
    if (clipPointsA.size() > 1) {
        ClipPoint* best = nullptr;
        
        // TODO: Set to zero and check result
        float maxDistance = -FLT_MAX;

        for (auto & cp : clipPointsA) {
            float distance = (clipPoints[0].p - cp.p).Norm();
            if (distance > maxDistance) {
                maxDistance = distance;
                best = &cp;
            }
        }
        
        clipPoints.push_back(*best);
    }

    // Get the largest triangle
    Vec3 maxCross;
    if (clipPointsA.size() > 2) {
        ClipPoint* best = nullptr;
        Vec3 v1 = clipPoints[1].p - clipPoints[0].p;
        
        // TODO: Set to zero and check result
        float maxDistance = -FLT_MAX;

        for (auto & cp : clipPointsA) {
            Vec3 cross = (clipPoints[0].p - cp.p).Cross(v1);
            float distance = cross.Norm();
            if (distance > maxDistance) {
                maxDistance = distance;
                maxCross = cross;
                best = &cp;
            }
        }
        
        clipPoints.push_back(*best);
    }

    // Get the largest triangle that attaches to the original triangle
    if (clipPointsA.size() > 3) {
        ClipPoint* best = nullptr;
        Vec3 v[3] = {
                clipPoints[1].p - clipPoints[0].p,
                clipPoints[2].p - clipPoints[1].p,
                clipPoints[0].p - clipPoints[2].p
        };

        // TODO: Set to zero and check result
        float maxDistance = -FLT_MAX;

        for (auto & cp : clipPointsA) {
            for (int i = 0; i < 3; ++i) {
                Vec3 cross = (clipPoints[i].p - cp.p).Cross(v[i]);
                float distance = cross.Norm();
                if (cross.Dot(maxCross) < 0 && distance > maxDistance) {
                    maxDistance = distance;
                    best = &cp;
                }
            }
        }
        
        if (best != nullptr) {
            clipPoints.push_back(*best);
        }
    }
}

size_t findFace(const Hull &hull, const Vec3 &normal) {
    auto &faces = hull.faces;
    auto it = max_element(faces.begin(), faces.end(),
            [&](const Face &a, const Face &b) { return a.normal.Dot(normal) > b.normal.Dot(normal); });
    return (size_t)(it - faces.begin());
}

vector<ClipPoint> clipPointsA;
vector<ClipPoint> clipPointsB;
vector<ClipPoint> clipPoints;

Physics::Physics() {
    clipPointsA.reserve(100);
    clipPointsB.reserve(100);
    clipPoints.reserve(100);
    temp.reserve(100);
}

void doContacts(Manifold &manifold, int iterations) {

    // TODO: keep a,b as pointers
    Body &a = *manifold.body[FIRST];
    Body &b = *manifold.body[SECOND];

    if ((a.sleepTimer > 0.0f || a.isFixed()) && (b.sleepTimer > 0.0f || b.isFixed())) {
        return;
    }

    size_t contactsCount = 0;
    array<Contact, 8> contacts;

    Vec3 axis = manifold.axis;

    Simplex s = manifold.simplex;
    doGJK(a.transform, b.transform, a.hull, b.hull, s, axis, iterations, manifold);
    SupportPoint sp = s[0];

    if (s.count == 4) {
        doEPA(a.transform, b.transform, a.hull, b.hull, s);
        axis = faceNormal(s[0].p, s[1].p, s[2].p).Dir(b.transform.pos - a.transform.pos);
        sp = s[0];
    }

    if (s.count == 4) {
        axis = manifold.axis;
        sp = findSeparatingAxis(a.transform, b.transform, a.hull, b.hull, axis, manifold.sps, manifold.spsCount, manifold);
    }

    axis.Normalize();
    //axis = Vec3(0,0,1).Dir(b.transform.pos - a.transform.pos);

    manifold.axis = axis;
    manifold.simplex = s;

    // find the collision plane in between the faces
    float distance = (-sp.p).Dot(axis);
    
    if (distance > 2.0f * Physics::COLLISION_OFFSET + Physics::SLOP) {
        manifold.contactsCount = 0;
        return;
    }
    
    Vec3 center = a.transform * a.hull.vertices[sp.i] + 0.5f * distance * axis;
    Face clipPlane = { center, axis };

    // Face of each object that is the most in the normal direction of the simplex
    size_t faceIndexA = findFace(a.hull, a.transform.rotateInv(-axis));
    size_t faceIndexB = findFace(b.hull, b.transform.rotateInv(axis));

    Face &faceA = a.hull.faces[faceIndexA];
    Face &faceB = b.hull.faces[faceIndexB];

    // TODO: Use normal from GJK reference face
    Vec3 normal = axis;
    Vec3 tangent = Geometry::getTangent(normal);
    Vec3 bitangent = normal.Cross(tangent);

    // Create the polygon of face A in the collision plane P
    clipPointsA.clear();
    clipPointsB.clear();
    
    
    generateClipPointsFromFace(clipPointsA, a, clipPlane, faceIndexA);
    generateClipPointsFromFace(clipPointsB, b, clipPlane, faceIndexB);

    polygonPolygonClipping(clipPointsA, clipPointsB, clipPlane);

    for(auto it = clipPointsA.begin(); it != clipPointsA.end(); ) {
        Vec3 posA = it->p - (it->p - a.transform * faceA.pos).Dot(a.transform.rotate(faceA.normal)) * a.transform.rotate(faceA.normal) + Physics::COLLISION_OFFSET * normal;
        Vec3 posB = it->p - (it->p - b.transform * faceB.pos).Dot(b.transform.rotate(faceB.normal)) * b.transform.rotate(faceB.normal) - Physics::COLLISION_OFFSET * normal;

        float distance = (posB - posA).Dot(normal);
        
        if (distance > Physics::SLOP) {
            it = clipPointsA.erase(it);
        } else {
            it++;
        }
    }

    // Clip point reduction
    reduceContacts(&manifold, normal, faceA, faceB, clipPoints, clipPointsA);

    for (auto & cp : clipPoints) {
        Contact &contact = contacts[contactsCount++];
        contact.clipPoint = cp;
        
        Vec3 posA = cp.p - (cp.p - a.transform * faceA.pos).Dot(a.transform.rotate(faceA.normal)) * a.transform.rotate(faceA.normal) + Physics::COLLISION_OFFSET * normal;
        Vec3 posB = cp.p - (cp.p - b.transform * faceB.pos).Dot(b.transform.rotate(faceB.normal)) * b.transform.rotate(faceB.normal) - Physics::COLLISION_OFFSET * normal;

        float distance = (posB - posA).Dot(normal);
        
        if (distance > Physics::SLOP) {
            --contactsCount;
            continue;
        }

        contact.distance = distance;
        contact.isSleeping = false;
        contact.sleepTimer = 0.0f;
        contact.pos[FIRST] = posA - a.transform.pos;
        contact.pos[SECOND] = posB - b.transform.pos;
        contact.pos0[FIRST] = -a.transform * posA;
        contact.pos0[SECOND] = -b.transform * posB;
        contact.n[NORMAL] = normal;
        contact.n[TANGENT] = tangent;
        contact.n[BINORMAL] = bitangent;
        contact.velocityBias = 0;

        contact.impulse[NORMAL] = 0.0f;
        contact.impulse[TANGENT] = 0.0f;
        contact.impulse[BINORMAL] = 0.0f;
        contact.correction = 0;
        contact.isStatic = false;

        size_t bestIndex = -1;
        float bestDistance = FLT_MAX;
        
        for (size_t i = 0; i < manifold.contactsCount; i++) {
            const Contact &oldContact = manifold.contacts[i];
            Vec3 oldContactPos = (a.transform * oldContact.pos0[FIRST] + b.transform * oldContact.pos0[SECOND]) * 0.5f;
            Vec3 newContactPos = (a.transform * contact.pos0[FIRST] + b.transform * contact.pos0[SECOND]) * 0.5f;

            float distance = (oldContactPos - newContactPos).Norm(); //(contact.clipPoint.p - oldContact.clipPoint.p).Norm();
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = i;
            }
        }

        if (bestIndex != -1 && bestDistance < 0.05f) {
            const Contact &oldContact = manifold.contacts[bestIndex];
            contact.impulse[NORMAL] = oldContact.impulse[NORMAL];
            contact.impulse[TANGENT] = oldContact.impulse[TANGENT];
            contact.impulse[BINORMAL] = oldContact.impulse[BINORMAL];
            //contact.correction = oldContact.correction;
            contact.velocityBias = oldContact.velocityBias;
        } else {
            //printf("no match: %p\n", &cp);
        }

    }

    if (contactsCount > 0) {
        //printf("axis: (%f, %f, %f)\n", axis.x, axis.y, axis.z);
    }

    manifold.contacts = contacts;
    manifold.contactsCount = contactsCount;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SOLVER
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void applyImpulse(Body &a, float i, Vec3 &n, Vec3 &I) {
    a.velocity += i * a.invMass * n;
    a.angularVelocity += i * I;
}

void applyImpulse(Body &a, Body &b, float i, Vec3 &n, Vec3 &Ia, Vec3 &Ib)       { applyImpulse(a, -i, n, Ia); applyImpulse(b, i, n, Ib); }

float bodySign(size_t b) { return b == FIRST ? -1.0f : 1.0f; }

Vec3 calculateVelocity(const Body &body, const Vec3 &pos) { return body.velocity + body.angularVelocity.Cross(pos); }

// Pre-calculates everything that is velocity-independent
void preSolveManifold(Manifold &manifold) {
    Body &a = *manifold.body[FIRST];
    Body &b = *manifold.body[SECOND];

    for (int i = 0; i < manifold.contactsCount; ++i) {
        auto &contact = manifold.contacts[i];
    
        for (int n = NORMAL; n < NORMALS_COUNT; ++n) {
            
            Vec3 localNormalA = a.transform.rotateInv(contact.n[n]);
            contact.inertia[FIRST][n] = a.transform.rotate(a.invInertia * contact.pos0[FIRST].Cross(localNormalA));

            Vec3 localNormalB = b.transform.rotateInv(contact.n[n]);
            contact.inertia[SECOND][n] = b.transform.rotate(b.invInertia * contact.pos0[SECOND].Cross(localNormalB));

            contact.mass[n] = 1.0f / (
                    a.invMass + contact.inertia[FIRST][n].Cross(contact.pos[FIRST]).Dot(contact.n[n]) +
                    b.invMass + contact.inertia[SECOND][n].Cross(contact.pos[SECOND]).Dot(contact.n[n]));
        }

        Vec3 vRel = calculateVelocity(b, contact.pos[SECOND]) - calculateVelocity(a, contact.pos[FIRST]);
        float velocity = vRel.Dot(contact.n[NORMAL]);

        float frictionIncrease = 1.0f + 0.5f * ((a.sleepTimer > 0.0f || a.isFixed()) && (b.sleepTimer > 0.0f || b.isFixed()));
        contact.friction = sqrtf(a.friction * b.friction * frictionIncrease);

        float restitution = max(a.restitution, b.restitution);

        contact.normalVelocity = velocity;
        contact.velocityBias = velocity < Physics::RESTITUTION_VELOCITY_THRESHOLD ? -velocity * restitution : 0.0f;
    }
}

void warmStart(Manifold &manifold) {
    Body &a = *manifold.body[FIRST];
    Body &b = *manifold.body[SECOND];

    for (int i = 0; i < manifold.contactsCount; ++i) {
        auto &contact = manifold.contacts[i];

        contact.impulse[TANGENT] *= 0.99f;
        contact.impulse[BINORMAL] *= 0.99f;
        contact.correction = 0.0f;

        applyImpulse(a, b, contact.impulse[NORMAL], contact.n[NORMAL], contact.inertia[FIRST][NORMAL], contact.inertia[SECOND][NORMAL]);
        applyImpulse(a, b, contact.impulse[TANGENT], contact.n[TANGENT], contact.inertia[FIRST][TANGENT], contact.inertia[SECOND][TANGENT]);
        applyImpulse(a, b, contact.impulse[BINORMAL], contact.n[BINORMAL], contact.inertia[FIRST][BINORMAL], contact.inertia[SECOND][BINORMAL]);
    }
}

void solveVelocities(Manifold &manifold, size_t iteration, float timeStep) {

    // TODO: Solve everything in one bang to make things bounce straighter

    Body &a = *manifold.body[FIRST];
    Body &b = *manifold.body[SECOND];


    if (a.velocity.Norm() < Physics::BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD &&
        a.angularVelocity.Norm() < Physics::BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD &&
        b.velocity.Norm() < Physics::BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD &&
        b.angularVelocity.Norm() < Physics::BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD) {
        a.velocity = 0;
        a.angularVelocity = 0;
        b.velocity = 0;
        b.angularVelocity = 0;
        return;
    }

    for (int i = 0; i < Physics::MANIFOLD_SOLVER_ITERATIONS; ++i) {
        manifold.isFixed = manifold.contactsCount > 0;

        // Tangent and binormal
        // TODO: Apply max friction to resultant vector of normal and bitangent friction (can't apply componentwise like we do now!)

        for (int i = 0; i < manifold.contactsCount; ++i) {
            auto &contact = manifold.contacts[i];

            for (int n = TANGENT; n <= BINORMAL; ++n) {
                float maxFriction = contact.friction * contact.impulse[NORMAL];
                float oldImpulse = contact.impulse[n];

                Vec3 vRel = calculateVelocity(b, contact.pos[SECOND]) - calculateVelocity(a, contact.pos[FIRST]);
                float velocity = vRel.Dot(contact.n[n]);
                const float targetVelocity = 0.0f;
                float impulse = (targetVelocity - velocity) * contact.mass[n];

                float sum = impulse + contact.impulse[n] * 0.99f;
                if (abs(sum) > maxFriction || abs(sum) > maxFriction) {
                    manifold.isFixed = false;
                }

                float clampedImpulse = max(min(sum, maxFriction), -maxFriction);
                
                contact.impulse[n] = clampedImpulse;
                float delta = clampedImpulse - oldImpulse;
                applyImpulse(a, b, delta, contact.n[n], contact.inertia[FIRST][n], contact.inertia[SECOND][n]);
            }
        }

        for (int i = 0; i < manifold.contactsCount; ++i) {
            auto &contact = manifold.contacts[i];
            
            // Normal impulse
            float oldImpulse = contact.impulse[NORMAL];

            Vec3 vRel = calculateVelocity(b, contact.pos[SECOND]) - calculateVelocity(a, contact.pos[FIRST]);
            float velocity = vRel.Dot(contact.n[NORMAL]);

            float targetVelocity = contact.velocityBias;

            float impulse = (targetVelocity - velocity ) * contact.mass[NORMAL];
            
            float sum = impulse + contact.impulse[NORMAL];
            
            float clampedImpulse = max(sum, 0.0f);
            
            contact.impulse[NORMAL] = clampedImpulse;
            float delta = clampedImpulse - oldImpulse;
            applyImpulse(a, b, delta, contact.n[NORMAL], contact.inertia[FIRST][NORMAL], contact.inertia[SECOND][NORMAL]);
        }
    }

    if (a.velocity.Norm() < Physics::BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD) {
        a.velocity = 0;
    }
    if (a.angularVelocity.Norm() < Physics::BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD) {
        a.angularVelocity = 0;
    }
    if (b.velocity.Norm() < Physics::BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD) {
        b.velocity = 0;
    }
    if (b.angularVelocity.Norm() < Physics::BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD) {
        b.angularVelocity = 0;
    }

}

void solvePosition(Manifold &manifold) {
    for (int i = 0; i < Physics::MANIFOLD_SOLVER_ITERATIONS; ++i) {
        for (int i = 0; i < manifold.contactsCount; ++i) {
            auto &contact = manifold.contacts[i];

            Vec3 dRel = (manifold.body[SECOND]->transform * contact.pos0[SECOND]) - (manifold.body[FIRST]->transform * contact.pos0[FIRST]);

            float oldCorrection = contact.correction;
            float distance = dRel.Dot(contact.n[NORMAL]);
            const float targetDistance = 0.0f;

            float correction = 0.8f * (targetDistance - distance) * contact.mass[NORMAL];
            float sum = correction + contact.correction;
            
            // Pull objects apart if there is penetration and pull objects toghether
            // to make surfaces align. Only pull the surfaces toghether when there is
            // an impulse in the normal direction to avoid objects sticking together like glue.
            float clampedCorrection = max(sum, 0.0f); //min(0.0f, 100.0f * -contact.impulse[NORMAL] * contact.mass[NORMAL]));
            
            contact.correction = clampedCorrection;
            float delta = clampedCorrection - oldCorrection;
            
            manifold.body[FIRST]->transform.pos -= contact.n[NORMAL] * delta * manifold.body[FIRST]->invMass;
            manifold.body[SECOND]->transform.pos += contact.n[NORMAL] * delta * manifold.body[SECOND]->invMass;

            manifold.body[FIRST]->transform.rot = 0.5f * Ternion(-delta * contact.inertia[FIRST][NORMAL]) * manifold.body[FIRST]->transform.rot;
            manifold.body[SECOND]->transform.rot = 0.5f * Ternion(delta * contact.inertia[SECOND][NORMAL]) * manifold.body[SECOND]->transform.rot;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ISLANDS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class T> void findIsland(Body* body, T callback) {

    if (body->isPartOfIsland || body->isFixed()) {
        return;
    }
    
    body->isPartOfIsland = true;
    callback(body);
    
    for(auto manifold : body->manifolds) {
        if (manifold->contactsCount > 0) {
            Body* other = (manifold->body[FIRST] == body) ? manifold->body[SECOND] : manifold->body[FIRST];
            findIsland(other, callback);
        }
    }
}

// TODO: Move to utils class ("swapPopRemove")
template <typename T> inline void swapRemove(vector<T> &v, const T &k)                                                  { *std::find(v.begin(), v.end(), k) = v.back(); v.pop_back(); }
template <typename T> inline void swapRemove(vector<T> &v, const typename vector<T>::iterator &it)                      { *it = v.back(); v.pop_back(); }
template <typename T> inline bool contains(vector<T> &v, const T &k)                                                    { return std::find(v.begin(), v.end(), k) != v.end(); }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// PUBLIC INTERFACE
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Use GJK instead of looking over all faces
Body* Physics::rayPicking(Vec3 &point, const Ray &ray) {

    Body* bestBody = nullptr;
    float bestDistance = FLT_MAX;
    for (auto &body : bodies) {
        
        if (body.isFixed()) {
            continue;
        }
        
        const Ray localRay = -body.transform * ray;

        if (Geometry::raySphereIntersection(ray, body.transform.pos, body.radius)) {

            for (size_t i = 0; i < body.hull.faces.size(); ++i) {
                const Face &face = body.hull.faces[i];
                
                // If face is pointiging away, skip it!
                if (face.normal.Dot(localRay.direction) >= 0) {
                    continue;
                }

                float distance = Geometry::rayPlaneIntersectionDistance(localRay, face);

                if (distance < 0 || distance > bestDistance) {
                    continue;
                }

                Vec3 projection = ray.origin + ray.direction * distance;
                Vec3 localProjection = -body.transform * projection;

                // Check if projection inside face region
                const auto &faceVertices = body.hull.faceVertices[i];
                const auto &faceEdgeNormals = body.hull.faceEdgeNormals[i];
                for (size_t j = 0; j < faceVertices.size(); ++j) {
                    Vec3 edgeNormal = faceEdgeNormals[j];
                    Vec3 edgePoint = body.hull.vertices[faceVertices[j]];
                    
                    if ((localProjection - edgePoint).Dot(edgeNormal) > 0) {
                        goto NEXT_FACE;
                    }
                }

                bestDistance = distance;
                bestBody = &body;
                point = projection;
                
                NEXT_FACE:;
            }
        }
    }
    
//    drawPoint(ray.origin, 0xFF0000FF, 10.0f);
//    drawPoint(point, 0xFF0000FF, 10.0f);
//    drawLine(ray.origin, point, 0xFF0000FF, 10.0f);
    
    return bestBody;
}


// Must not be called during call to step
void Physics::add(Body &body) {
    bodies.push_back(body);
}

// Must not be called during call to step
void Physics::remove(Body &body) {
    // TODO: Implement reference scheme for bodies
}

// Must not be called during call to step
void Physics::clearAll() {
    bodies.clear();
    manifolds.clear();
}

void Physics::setPaused(const bool p) {
    paused = p;
}

void setColor(GLint color) {
    float c[4] = {
        ((color >> 24) & 0xFF) / 255.0f,
        ((color >> 16) & 0xFF) / 255.0f,
        ((color >>  8) & 0xFF) / 255.0f,
        ((color >>  0) & 0xFF) / 255.0f};

    glColor3f(c[0], c[1], c[2]);
}

void drawPoint(GLint color, const Vec3& point, GLfloat size) {
    glPointSize(size);
    glDisable(GL_LIGHTING);
    setColor(color);
    glPushMatrix();
        glBegin(GL_POINTS);
            glVertex3fv(point);
        glEnd();
    glPopMatrix();
    glEnable(GL_LIGHTING);
}

void drawLine(Vec3 v0, Vec3 v1, GLint color, float width) {
    glDisable(GL_LIGHTING);
    setColor(color);
    glLineWidth(width);
    glBegin(GL_LINES);
        glVertex3fv(v0);
        glVertex3fv(v1);
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawLine(Vec3 v0, Vec3 v1, GLint color) {
    drawLine(v0, v1, color, 2.0f);
}


void Physics::drawManifolds() {
    for (auto &manifold : manifolds) {
        auto a = manifold.body[FIRST];
        auto b = manifold.body[SECOND];
        for (int i = 0; i < manifold.contactsCount; i++) {
            auto &contact = manifold.contacts[i];
            int intensity = max(min(255, (int)(abs(contact.impulse[NORMAL]) * 100000.0f)), 0);
            int color2 = (intensity << 16) + (intensity << 8) + (intensity << 24);
            /*
            int color = ((int)(max(0.0f, contact.impulse[NORMAL]) * 100000.0f)) << 16;
                color += ((int)(max(0.0f, contact.impulse[TANGENT]) * 5000000.0f)) << 8;
                color += ((int)(max(0.0f, contact.impulse[BINORMAL]) * 5000000.0f)) << 24;
                */
            
            int color = color2;
            
            // drawPoint(color, a->transform * contact.pos0[FIRST], 15.0f);
            // drawPoint(color, b->transform * contact.pos0[SECOND], 15.0f);
        }

//        drawLine(a->transform.pos, a->transform.pos + manifold.axis * 5, 0x00FFFFFF, 2.0f);
//        drawLine(b->transform.pos, b->transform.pos - manifold.axis * 5, 0x00FFFFFF, 2.0f);
        
    }
}

void Physics::drawDebug() {
    
    for (auto &manifold : manifolds) {
        auto a = manifold.body[FIRST];
        auto b = manifold.body[SECOND];
        for (int i = 0; i < manifold.contactsCount; i++) {
            auto &contact = manifold.contacts[i];
            int intensity = max(min(255, (int)(abs(contact.impulse[NORMAL]) * 100000.0f)), 0);
            int color2 = (intensity << 16) + (intensity << 8) + (intensity << 24);
            /*
            int color = ((int)(max(0.0f, contact.impulse[NORMAL]) * 100000.0f)) << 16;
                color += ((int)(max(0.0f, contact.impulse[TANGENT]) * 5000000.0f)) << 8;
                color += ((int)(max(0.0f, contact.impulse[BINORMAL]) * 5000000.0f)) << 24;
                */
            
            int color = color2;
            float mul = 1.0f;
            
            if (contact.n[NORMAL].z < -0.8f) {
                color = 0xFFFF00FF;
                mul = -1.0f;
            }
            
            drawPoint(color, a->transform * contact.pos0[FIRST], 15.0f);
            drawPoint(color, b->transform * contact.pos0[SECOND], 15.0f);
            drawLine(a->transform * contact.pos0[FIRST], a->transform * contact.pos0[FIRST] + (
            contact.n[NORMAL] * contact.impulse[NORMAL] * mul +
            contact.n[TANGENT] * contact.impulse[TANGENT] +
            contact.n[BINORMAL] * contact.impulse[BINORMAL]
            )* 10000.0f, color);

        }
    }
}

void Physics::step(float timeStep, const Physics::CollisionCallbacks &callback) {

    if (paused) {
        return;
    }
    
    // Update velocities
    const Vec3 gravity(0.0f, 0.0f, -GRAVITY);
    for (auto &body : bodies) {
        body.totalVelocity = FLT_MIN;
        body.totalAngularVelocity = FLT_MIN;

        if (body.isAwake && !body.isFixed()) {
            body.velocity += gravity * timeStep;
        }
    }
    
    doBroadPhase(bodies, bodies.size(), timeStep, [&] (Body* a, Body* b) {
        if (std::find(a->proxmimities.begin(), a->proxmimities.end(), b) == a->proxmimities.end()) {
            manifolds.push_back({ a, b });
            a->proxmimities.push_back(b);
            b->proxmimities.push_back(a);
            a->manifolds.push_back(&manifolds.back());
            b->manifolds.push_back(&manifolds.back());
        }
    });
    
    // Remove manifolds that no longer overlap
    for (auto it = manifolds.begin(); it != manifolds.end(); ) {
        
        Body* a = it->body[FIRST];
        Body* b = it->body[SECOND];
        
        if(!(a->isFixed() && b->isFixed()) && (a->isAwake || b->isAwake) && !testBroadPhaseProximity(a, b, timeStep)) {
            a->proxmimities.remove(b);
            b->proxmimities.remove(a);
            a->manifolds.remove(&(*it));
            b->manifolds.remove(&(*it));
            it = manifolds.erase(it);

            // Call the contact ended callback if needed
            if (it->contactStartedCalled == true && it->ignoreContacts == false) {
                callback.contactEnded(a, b);
            }
        } else {
            ++it;
        }
    }

    // Narrowphase
    for (auto &manifold : manifolds) {

        if (!manifold.ignoreContacts) {
            doContacts(manifold, iterations);
        }

        // Call the callbacks
        if (manifold.contactsCount > 0 && !manifold.contactStartedCalled) {
            manifold.ignoreContacts = !callback.contactStarted(manifold.body[FIRST], manifold.body[SECOND]);
            if (manifold.ignoreContacts) {
                manifold.contactsCount = 0;
            }
            manifold.contactStartedCalled = true;
        } else if (manifold.contactsCount == 0 && manifold.contactStartedCalled) {
            callback.contactEnded(manifold.body[FIRST], manifold.body[SECOND]);
            manifold.contactStartedCalled = false;
            manifold.ignoreContacts = false;
        }
    }

    // Find and solve islands
    for(auto &body : bodies) {
        body.isPartOfIsland = false;
    }

    for(auto &manifold : manifolds) {
        manifold.isPartOfIsland = false;
    }

    vector<Body*> islandBodies;
    vector<Manifold*> islandManifolds;

    for (auto &body : bodies) {
        if (!body.isPartOfIsland && !body.isFixed() && body.isAwake) {
            islandBodies.clear();
            islandManifolds.clear();
            
            bool isIslandAwake = false;
            
            findIsland(&body, [&](Body* body) {
                islandBodies.push_back(body);
                isIslandAwake = isIslandAwake || body->isAwake;
                
                // TODO: Nicer way to write this?
                for (auto* manifold : body->manifolds) {
                    if (!manifold->isPartOfIsland) {
                        manifold->isPartOfIsland = true;
                        islandManifolds.push_back(manifold);
                    }
                }
            });
            
            if (!isIslandAwake) {
                continue;
            } else {
                for (auto* body : islandBodies) {
                    body->isAwake = true;
                }
            }

            // Solve island
            for (auto* manifold : islandManifolds) {
                if (manifold->body[FIRST]->isAwake || manifold->body[SECOND]->isAwake) {
                    preSolveManifold(*manifold);
                }
            }

            for (auto* manifold : islandManifolds) {
                if (manifold->body[FIRST]->isAwake || manifold->body[SECOND]->isAwake) {
                    warmStart(*manifold);
                }
            }

            /*
            for (auto &joint : islandJoints) {
                joints[i].presolve();
            }
            */

            for (int k = 0; k < VELOCITY_SOLVER_ITERATIONS; k++) {
                for (auto* manifold : islandManifolds) {
                    solveVelocities(*manifold, k, timeStep);
                }
                
                /*
                for (auto &joint : islandJoints) {
                    joints[i].solveVelocities();
                }
                */
            }

            for (auto* manifold : islandManifolds) {
                auto a = manifold->body[FIRST];
                auto b = manifold->body[SECOND];
                for (int i = 0; i < manifold->contactsCount; i++) {
                    auto &contact = manifold->contacts[i];
                    for (int n = NORMAL; n <= BINORMAL; n++) {

                        if (!a->isFixed() && a->isAwake) {
                            float velocityA = contact.impulse[n] * a->invMass;
                            Vec3 angularVelocityA = contact.impulse[n] * contact.inertia[FIRST][n];

                            a->totalVelocity += velocityA * velocityA;
                            a->totalAngularVelocity += angularVelocityA.Norm();
                        }

                        if (!b->isFixed() && b->isAwake) {
                            float velocityB = contact.impulse[n] * b->invMass;
                            Vec3 angularVelocityB = contact.impulse[n] * contact.inertia[SECOND][n];

                            b->totalVelocity += velocityB * velocityB;
                            b->totalAngularVelocity += angularVelocityB.Norm();
                        }
                    }
                }
            }

            // Put island to sleep
            if (ENABLE_SLEEPING) {
                float sleepTimer = FLT_MAX;
                for(auto* body : islandBodies) {
                    bool isNotMoving = body->velocity.Norm() < BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD;
                    bool isNotRotating = body->angularVelocity.Norm() < BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD;
                    
                    body->sleepTimer = isNotMoving && isNotRotating ? body->sleepTimer + timeStep : 0;
                    sleepTimer = min(sleepTimer, body->sleepTimer);
                }
/*
                if (sleepTimer > 60.0f) {
                    for(auto* body : islandBodies) {
                        body->isAwake = false;
                    }
                }
*/
            }
        }
    }

    // Update positions
    for (auto &body : bodies) {
        if (!body.isFixed()) {
            //body.velocity *= max(0.7f, min(1.0f, 10.0f * sqrt(body.velocity.Norm() / body.totalVelocity)));
            body.transform.pos += body.velocity * timeStep;
            
            //body.angularVelocity *= max(0.7f, min(1.0f, 10.0f * sqrt(body.angularVelocity.Norm() / body.totalAngularVelocity)));
            body.transform.rot = 0.5f * timeStep * Ternion(body.angularVelocity) * body.transform.rot;
        }
    }
    
    // Position Solver
    for (int k = 0; k < POSITION_SOLVER_ITERATIONS; k++) {
        for (auto &manifold : manifolds) {
            auto a = manifold.body[FIRST];
            auto b = manifold.body[SECOND];

            if ((a->sleepTimer > 60.0f || a->isFixed()) && (b->sleepTimer > 60.0f || b->isFixed())) {
                continue;
            }

            solvePosition(manifold);
        }
    }
}
}
}
