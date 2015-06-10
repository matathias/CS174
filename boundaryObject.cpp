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
        
    int ind = 0;
    int sign = 1;
    Vector3d direc(0, 0, 0);
    switch (bot) {
        case GROUND:
            direc(1) = 1;
            ind = 1;
            break;
        case CEILING:
            direc(1) = -1;
            ind = 1;
            break;
        case WALL_LEFT:
            direc(0) = 1;
            ind = 0;
            break;
        case WALL_RIGHT:
            direc(0) = -1;
            ind = 0;
            break;
        case WALL_FRONT:
            direc(2) = 1;
            ind = 2;
            break;
        case WALL_BACK:
            direc(2) = -1;
            ind = 2;
            break;
        default:
            direc(0) = 0;
            break;
    }
    Vector3d point = this->getFarthestPointInDirection(direc);
    boundary = point(ind);
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

float BoundaryObject::getBoundary()
{
    return boundary;
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
