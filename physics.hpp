// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#ifndef MINI3D_PHYSICS_PHYSICS_H
#define MINI3D_PHYSICS_PHYSICS_H

#include "types.hpp"
#include "geometry.hpp"
#include "constraints.hpp"

#include <list>

namespace mini3d {
namespace physics {

class Physics {

public:

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CONSTANTS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// WORLD
static constexpr float GRAVITY = 0.004f;

// FEEATURE SWITCHES
static const bool ENABLE_SLEEPING = true;

// COLLISION DETECTION
static constexpr float COLLISION_EPSILON = 0.001f; // Contacts will be calculated within 1 mm
static constexpr float COLLISION_EPSILON_SQUARED = COLLISION_EPSILON * COLLISION_EPSILON;
static constexpr float COLLISION_OFFSET = 0.01f; // Margin around objects to avoid running EPA too often.

// SOLVER
static constexpr size_t VELOCITY_SOLVER_ITERATIONS = 10;
static constexpr size_t POSITION_SOLVER_ITERATIONS = 10;
static constexpr size_t MANIFOLD_SOLVER_ITERATIONS = 3;
static constexpr float SLOP = 0.01f; // The size of the floaty region around objects that dampen jitter
static constexpr float RESTITUTION_VELOCITY_THRESHOLD = 5.0f * -Physics::GRAVITY; // Below this collision velocity collisions will be fully elastic (no restitution)
 
// SLEEPING
static constexpr float BODY_SLEEP_TIME = 10.0f;
static constexpr float BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD =  0.0000001f;
static constexpr float BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD = 0.0000001f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ARRAYS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Hide all of this!
list<Body> bodies;
vector<MouseConstraint> joints;

Physics();


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// COLLISION CALLBACKS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CollisionCallbacks {
    // return true if the collison should be processed
    virtual bool contactStarted(const Body* a, const Body* b) const { return true; }
    virtual void contactEnded(const Body* a, const Body* b) const { }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// PHYSICS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Find closest body in direction of ray
Body* rayPicking(Vec3 &point, const Ray &ray);

// Must not be called during call to step
void add(Body &body);

// Must not be called during call to step
void remove(Body &body);

// Must not be called during call to step
void clearAll();

// Steps the simulation forward by time step
void step(float timeStep, const Physics::CollisionCallbacks &callback);
    

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DEBUG
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
void setPaused(const bool paused);

void drawManifolds();

void drawDebug();

};
}
}
#endif /* MINI3D_PHYSICS_PHYSICS_H */
