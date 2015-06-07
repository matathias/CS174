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

vector<HE*>* getHalfEdges()
{
    return halfEdges;
}

vector<HEF*>* getHalfFaces()
{
    return halfFaces;
}

vector<HEV*>* getHalfVertices()
{
    return halfVertices;
}

vector<Vector3d>* getNormals()
{
    return normals;
}

// Collision test function
bool Object::collidedWith(Object *o)
{
    /* TODO Implement collision detection here */
    return false;
}

// Fill the object's half-edge data structure
void intializeHalfEdge(MatrixXd scl, MatrixXd rot, Vector3d pos)
{
    float du = 2 * M_PI / (float) Nu;
    float dv = M_PI / (float) Nv;
    vector<Vector3d> *pts = new vector<Vector3d>;
    vector<Vector3d> *faces = new vector<Vector3d>;

    // Find all of the 3d points based on the (u,v) parametric values.
    for (int i = 1; i <= Nu; i++)
    {
        for (int j = 0; j <= Nv; j++)
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
    for (int i = 0; i < Nu; i++)
    {
        for (int j = 0; j <= Nv; j++)
        {
            Vector3d face1(0,0,0);
            Vector3d face2(0,0,0);

            ind1 = (ind1 + 1) % (Nu * (Nv+1));
            if (ind1 % (Nv+1) != Nv)
            {
                int ind2 = (ind1 + 1) % (Nu * (Nv+1));
                int ind3 = (ind1 + Nv+1) % (Nu * (Nv+1));
                int ind4 = (ind1 + 1 + Nv+1) % (Nu * (Nv+1));

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
    createHalfVertices(pts, halfVertices);
    createHalfOthers(faces, halfEdges, halfFaces);
    int successful = orientFace(halfFaces->at(0), halfFaces, halfVertices);
    if(successful != 0)
    {
        cerr << "Non-orientable surface." << endl;
        exit(1);
    }

    findAllVertexNormals(normals, halfVertices);
}
