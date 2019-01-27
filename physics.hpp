// Copyright (c) <2018> Daniel Peterson
// This file is part of Mini3D <www.mini3d.org>
// It is distributed under the MIT Software License <www.mini3d.org/license.php>

#ifndef MINI3D_PHYSICS_PHYSICS_H
#define MINI3D_PHYSICS_PHYSICS_H

#include "types.hpp"
#include "geometry.hpp"
#include "constraints.hpp"

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
static constexpr float COLLISION_EPSILON = 0.01f; // Contacts will be calculated within 1 mm
static constexpr float COLLISION_EPSILON_SQUARED = COLLISION_EPSILON * COLLISION_EPSILON;
static constexpr float COLLISION_OFFSET = 0.01f; // Margin around objects to avoid running EPA too often.

// SOLVER
static constexpr size_t VELOCITY_SOLVER_ITERATIONS = 8;
static constexpr size_t POSITION_SOLVER_ITERATIONS = 4;
static constexpr size_t MANIFOLD_SOLVER_ITERATIONS = 2;
static constexpr float SLOP = 0.01f; // The size of the floaty region around objects that dampen jitter
static constexpr float MAX_CONTACT_NEGATIVE_VELOCITY = Physics::SLOP * -0.0001f; // Makes objects sink into eachother for stable stacking
static constexpr float RESTITUTION_VELOCITY_THRESHOLD = -Physics::GRAVITY * 20.0f; // Below this collision velocity collisions will be fully elastic (no restitution)
 
// SLEEPING
static constexpr float BODY_SLEEP_TIME = 30.0f;
static constexpr float BODY_SLEEP_LINEAR_VELOCITY_THRESHOLD = 0.0000005f;
static constexpr float BODY_SLEEP_ANGULAR_VELOCITY_THRESHOLD = 0.0000005f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ARRAYS
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Hide!
vector<Body*> bodies;
vector<MouseConstraint> joints;


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
void add(Body* body);

// Must not be called during call to step
void remove(Body* body);

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
    
};
}
}
#endif /* MINI3D_PHYSICS_PHYSICS_H */
