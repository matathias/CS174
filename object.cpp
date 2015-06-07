/* Class file for objects.
 * 
 * This file contains the implementation for a basic object. This object 
 * contains no information about its physics, only about its shape and location.
 * This is useful for things like static floors and walls.
 *
 * Physical objects can use this class as a superclass.
 *
 * Something something OOP.
 */
 
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"

/********** Member function definitions **********/
// Constructor
Object::Object(MatrixXd scl, MatrixXd rot, Vector3d pos, float ee, float nn)
{
    position = pos;
    scale = scl;
    rotate = rot;
    
    e = ee;
    n = nn;
    
    physical = false;
}

// Manipulators
void Object::setPosition(Vector3d pos)
{
    position = pos;
}

void Object::setScale(MatrixXd scl)
{
    scale = scl;
}

void Object::setRotate(MatrixXd rot)
{
    rotate = rot;
}

void Object::setE(float ee)
{
    e = ee;
}

void Object::setN(float nn)
{
    n = nn;
}

// "Get" functions
Vector3d Object::getPosition()
{
    return position;
}

MatrixXd Object::getScale()
{
    return scale;
}

MatrixXd Object::getRotate()
{
    return rotate;
}

float Object::getE()
{
    return e;
}

float Object::getN()
{
    return n;
}

bool Object::isPhysical()
{
    return physical;
}

// Collision test function
bool Object::collisionTest(Object *o)
{
    /* TODO Implement collision detection here */
    return false;
}
