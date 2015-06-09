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
#include <math.h>
#define _USE_MATH_DEFINES
#include <float.h>
#include <stdio.h>

// parameterization constants; the higher they are, the more polygons objects
// will have
#define NU 80
#define NV 80

#include "object.h"
#include "util.h"

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
    
    initializeHalfEdge(scl, rot, pos);
}

// Manipulators
void Object::setPosition(Vector3d pos)
{
    Vector3d translation = pos - position;
    position = pos;
    updateVerticesPos(translation);
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

vector<HE*>* Object::getHalfEdges()
{
    return &halfEdges;
}

vector<HEF*>* Object::getHalfFaces()
{
    return &halfFaces;
}

vector<HEV*>* Object::getHalfVertices()
{
    return &halfVertices;
}

vector<Vector3d>* Object::getNormals()
{
    return &normals;
}

// Collision detection function (makes use of the GJK Algorithm)
bool Object::collidedWith(Object *o)
{
    // Get the initial direction, which will be the vector between the two
    // objects' centers
    Vector3d d = position - o->getPosition();
    // Create the simplex vector and add the first point to it
    vector<Vector3d> simplex;
    // Get the first support vector
    Vector3d p1 = this->getFarthestPointInDirection(d);
    Vector3d p2 = o->getFarthestPointInDirection(d * -1);
    Vector3d p3 = p1 - p2;
    simplex.push_back(p3);
    // negate d
    d = d * -1;
    // start looping until we determine whether or not a collision has occured
    while(true) {
        // Add a new point to the simplex
        Vector3d p1 = this->getFarthestPointInDirection(d);
        Vector3d p2 = o->getFarthestPointInDirection(d * -1);
        Vector3d p3 = p1 - p2;
        simplex.push_back(p3);
        // make sure that this point actually passed the origin
        if (simplex.back().dot(d) <= 0) {
            // If this point is not past the origin then the objects have not
            // collided, due to properties of the Minkowski Sum
            return false;
        }
        else {
            // determine if the origin is within the current simplex. If it
            // isn't, then containsOrigin will modify d to contain a new
            // direction that will be used at the beginning of the loop
            if (containsOrigin(&simplex, &d)) {
                // Collision!
                return true;
            }
        }
    }
    
    // We should never get here, but just so the compiler never complains...
    return false;
}

// Fill the object's half-edge data structure
void Object::initializeHalfEdge(MatrixXd scl, MatrixXd rot, Vector3d pos)
{
    float du = 2 * M_PI / (float) NU;
    float dv = M_PI / (float) NV;
    vector<Vector3d> *pts = new vector<Vector3d>;
    vector<Vector3d> *faces = new vector<Vector3d>;

    // Find all of the 3d points based on the (u,v) parametric values.
    for (int i = 1; i <= NU; i++)
    {
        for (int j = 0; j <= NV; j++)
        {
            float u = -M_PI + du * i;
            float v = -M_PI / 2 + dv * j;
            Vector3d para = psq(u,v,(float)n,(float)e);
            para = scl * para;
            para = rot * para;
            para = pos + para;

            pts->push_back(para);
        }
    }

    // Because we added the points to pts in a straight-forward manner, we can
    // easily determine what the vertices of the faces should be using the
    // parameterization.
    int ind1 = 0;
    for (int i = 0; i < NU; i++)
    {
        for (int j = 0; j <= NV; j++)
        {
            Vector3d face1(0,0,0);
            Vector3d face2(0,0,0);

            ind1 = (ind1 + 1) % (NU * (NV+1));
            if (ind1 % (NV+1) != NV)
            {
                int ind2 = (ind1 + 1) % (NU * (NV+1));
                int ind3 = (ind1 + NV+1) % (NU * (NV+1));
                int ind4 = (ind1 + 1 + NV+1) % (NU * (NV+1));

                face1(0) = ind1;
                face1(1) = ind2;
                face1(2) = ind3;

                face2(0) = ind3;
                face2(1) = ind2;
                face2(2) = ind4;

                faces->push_back(face1);
                faces->push_back(face2);
            }
        }
    }

    // Store the points and faces to a half-edge data structure.
    createHalfVertices(pts, &halfVertices);
    createHalfOthers(faces, &halfEdges, &halfFaces);
    int successful = orientFace(halfFaces.at(0), &halfFaces, &halfVertices);
    if(successful != 0)
    {
        cerr << "Non-orientable surface." << endl;
        exit(1);
    }

    findAllVertexNormals(&normals, &halfVertices);
}

// Update the object's vertices with the given translation vector
void Object::updateVerticesPos(Vector3d trans)
{
    for (int i = 0; i < halfVertices.size(); i++) {
        HEV *vert = halfVertices.at(i);
        vert->x = vert->x + trans(0);
        vert->y = vert->y + trans(1);
        vert->z = vert->z + trans(2);
    }
}

// Return the vertex that is farthest in the direction given by d
// This function is used primarily by the collision detection algorithm
Vector3d Object::getFarthestPointInDirection(Vector3d d)
{
    Vector3d ret(halfVertices.at(0)->x, halfVertices.at(0)->y, 
                 halfVertices.at(0)->z);
    float magnitude = ret.dot(d);
    
    for (int i = 1; i < halfVertices.size(); i++) {
        Vector3d tmp(halfVertices.at(i)->x, halfVertices.at(i)->y,
                     halfVertices.at(i)->z);
        float dist = d.dot(tmp);
        if (dist >= magnitude) {
            ret = tmp;
            magnitude = dist;
        }
    }
    return ret;
}
