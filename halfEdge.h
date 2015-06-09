
#include <map>
#include <utility> // pair
#include <list>
#include <vector>
#include <algorithm> //swap
#include <cassert>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Eigen>

#pragma once
#include "util.h"
using namespace Eigen;
using namespace std;

// half edge structure
typedef struct HE {
    struct HE * brother; // the HE in the opp dir.
    struct HEF * adj_face; // the face
    struct HE * next; // the HE that is 'out' of 'vertex'
    struct HEV * vertex; // the vertex this HE points to
} HE;

typedef struct HEF {
    int a,b,c; // useful for helping the initial loading process.
    struct HE * adj; // one of the three HEs that surround a face
    int oriented; // also useful for the loading. marks if this face was oriented or not.
} HEF;

typedef struct HEV {
    double x;
    double y;
    double z;
    struct HE * out; // one of the two HEs whose brother points to this vertex
} HEV;

// Uses the Cantor pairing function to return a unique key
int findKey(int a, int b);

// Function to load the points into the halfVertices vector.
void createHalfVertices(vector<Vector3d> *pts, vector<HEV*> *halfVertices);

// Function to load the faces into the halfFaces and halfEdges vectors.
void createHalfOthers(vector<Vector3d> *f, vector<HE*> *halfEdges, vector<HEF*> *halfFaces);

// Orients all of the faces. If the function fails to orient all faces properly
// then it will return a non-zero value; otherwise it returns 0.
int orientFace(HEF *hef, vector<HEF*> *halfFaces, vector<HEV*> *halfVertices);

// Function to check whether or not brother edges were assigned correctly
int checkBrothers(int select, vector<HE*> *halfEdges);

// Calculates and returns the normal of the given face
Vector3d findFaceNormal(HEF *hef, vector<HEV*> *halfVertices);

// Calculates and returns the area of the given face. The face is assumed to be
// a triangle.
double findFaceArea(HEF *hef, vector<HEV*> *halfVertices);

// Returns a vector of all the faces incident to the given vertex.
vector<HEF*> * findIncidentFaces(HEV *hev);

// Returns the normal vector of the given vertex using area weighted normals.
Vector3d findVertexNormal(HEV *hev, vector<HEV*> *halfVertices);

// Finds the normal vector for every vertex in halfVertices and stores the
// normals in the global normals vector with index corresponding to its
// respective vertex.
void findAllVertexNormals(vector<Vector3d> * normals, vector<HEV*> *halfVertices);

// Uses the calculated normal vectors to find the bump map normals by
// multiplying each normal by a random value between 0 and 1.
void calcBumpNormals(vector<Vector3d> *normals, vector<Vector3d> *bumpNormals);
