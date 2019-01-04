//
//  constraints.hpp
//  contacttest
//
//  Created by Daniel Peterson on 2018-11-03.
//  Copyright © 2018 Daniel Peterson. All rights reserved.
//

#ifndef MINI3D_PHYSICS_CONTACTS_HPP
#define MINI3D_PHYSICS_CONTACTS_HPP

namespace mini3d {
namespace physics {


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ABSTRACT CONSTRAINT
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For defining custom constraints
struct Constraint {
    virtual void presolve() = 0;
    virtual void solveVelocities() = 0;
};
    

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MOUSE CONSTRAINT
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct MouseConstraint : public Constraint {
    Body *body;

    // Wolrd space
    Vec3 mousePoint;
    Vec3 normal;
    float distance;

    // Body space
    Vec3 contactPoint;
    Vec3 angularMass;
    float mass;
    
    virtual void presolve() {
        normal = (body->transform * contactPoint - mousePoint);
        distance = -normal.Length();
        if (normal != Vec3(0,0,0)) {
            normal.Normalize();
        }
        
        angularMass = body->transform.rotate(body->invInertia * contactPoint.Cross(body->transform.rotateInv(normal)));
        mass = 1.0f / (body->invMass + angularMass.Cross(body->transform.rotate(contactPoint)).Dot(normal));
        
        body->angularVelocity *= 0.95f;
    }
    
    virtual void solveVelocities() {
        Vec3 vRel = - body->velocity - body->angularVelocity.Cross(body->transform.rotate(contactPoint));
        float velocity = vRel.Dot(normal);
        
        float dampening = 0.005f * -velocity;
        
        // Calculate velocity bias for restitution
        float baumgarte = -min((distance + 0.01f) * 0.05f, 0.0f);
        float impulse = max((-velocity + baumgarte + dampening) * mass, 0.0f);

        body->velocity -= impulse * body->invMass * normal;
        body->angularVelocity -= impulse * angularMass;
    }
};
}
}

#endif /* MINI3D_PHYSICS_CONTACTS_HPP */
