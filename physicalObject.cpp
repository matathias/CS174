/* Class file for objects with physics.
 *
 * This is a child class of Object. Physical Objects contain physics information
 * such as mass and velocity.
 */

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"
#include "physicalObject.h"


/********** Member function definitions **********/
// Constructor
PhysicalObject::PhysicalObject(MatrixXd scl, MatrixXd rot, Vector3d pos, 
                               Vector3d rgb, float a, float ee, 
                               float nn, Vector3d vel, float m) : Object(scl, rot, pos, rgb, a, ee, nn)
{
    velocity = vel;
    mass = m;
    physical = true;
    hitstun_timer = 0;
}

// Manipulator functions
void PhysicalObject::setVelocity(Vector3d vel)
{
    velocity = vel;
}

void PhysicalObject::setMass(float m)
{
    mass = m;
}

// "Get" functions
Vector3d PhysicalObject::getVelocity()
{
    return velocity;
}

float PhysicalObject::getMass()
{
    return mass;
}

int PhysicalObject::get_hitstun()
{
    return hitstun_timer;
}

int PhysicalObject::set_hitstun()
{
    hitstun_timer = 5;
}

int PhysicalObject::reduce_hitstun()
{
    hitstun_timer--;
}
