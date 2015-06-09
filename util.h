/* CS/CNS 171 Fall 2014
 *
 * This header file contains some useful utility functions.
 */

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

/* Gets the translation matrix for a given translation vector (x, y, z). */
MatrixXd get_translate_mat(double x, double y, double z);
/* Gets the rotation matrix for a given rotation axis (x, y, z) and angle. */
MatrixXd get_rotate_mat(double x, double y, double z, double angle);
/* Gets the scaling matrix for a given scaling vector (x, y, z). */
MatrixXd get_scale_mat(double x, double y, double z);
/* Gets the perspective projection matrix given the frustum parameters. */
MatrixXd get_perspective_mat(double near, double far,
                             double left, double right,
                             double top, double bottom);

/* Takes a Vector3d and turns it into a Vector4d. */
Vector4d vector3to4(Vector3d vec);
/* Takes a Vector4d and turns it into a Vector3d. */
Vector3d vector4to3(Vector4d vec);

/* Takes a 3x3 MatrixXd and turns it into a 4x4 MatrixXd. */
MatrixXd matrix3to4(MatrixXd mat);
/* Takes a 4x4 MatrixXd and turns it into a 3x3 MatrixXd. */
MatrixXd matrix4to3(MatrixXd mat);

/* Parametric cosine analog function. */
double cosine(double u, double e);
/* Parametric sine analog function. */
double sine(double u, double e);
/* Parametric superquadric function. */
Vector3d psq(float u, float v, float n, float e);

/* Returns -1 for negative numbers, 1 for positive numbers, and 0 for zero. */
int sign(double s);

/* These functions exist primarily for use in the collision detection algorithm.
 */
Vector3d tripleProduct(Vector3d a, Vector3d b, Vector3d c);
// Returns true if the given simplex s contains the origin, and false otherwise
bool containsOrigin(vector<Vector3d> *s, Vector3d *d);

// These functions exist for use in halfEdge.cpp
/*
 */

// Returns the square of val
double square(double val);

// Returns the distance between a and b. Assumes a and b represent points (or
// the endpoints of the vectors, whatever)
double vectorDistance(Vector3d a, Vector3d b);

// Returns true if every value of a is zero
bool vecIsZero(Vector3d a);
