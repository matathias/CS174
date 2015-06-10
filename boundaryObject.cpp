/* Class file for boundary objects.
 *
 * This is a child class of Object. Boundary objects store what kind of
 * boundary they are - be it floor, wall, or ceiling. This makes it easy to
 * determine what velocity component to reverse on physical object collision.
 *
 * Additionally, every boundary object has a dampening value that is capped at
 * 1 - this value is multiplied by an object's velocity when that object hits
 * the boundary, to simulate the transfer of kinetic energy.
 */

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"
#include "boundaryObject.h"


/********** Member function definitions **********/
// Constructor
BoundaryObject::BoundaryObject(MatrixXd scl, MatrixXd rot, Vector3d pos, Vector3d rgb, 
                               float a, float ee, float nn, boundary_t b, float d) : 
                               Object(scl, rot, pos, rgb, a, ee, nn)
{
    bot = b;
    if (d <= 1 && d >= 0)
        dampening = d;
    else
        dampening = 0.9;
}

// Get functions
boundary_t BoundaryObject::getBoundaryType()
{
    return bot;
}

float BoundaryObject::getDampening()
{
    return dampening;
}

// Set functions
void BoundaryObject::setBoundaryType(boundary_t b)
{
    bot = b;
}

void BoundaryObject::setDampening(float d)
{
    if (d <= 1 && d >= 0)
        dampening = d;
}
