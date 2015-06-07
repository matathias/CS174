/* CS/CNS 171 Fall 2014
 *
 * This header file contains some useful utility functions.
 */

#pragma once

#include "main.hpp"

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

/* Implicit Superquadric function. */
double isq(Vector3d vec, double e, double n);
/* Parametric cosine analog function. */
double cosine(double u, double e);
/* Parametric sine analog function. */
double sine(double u, double e);
/* Parametric superquadric function. */
Vector3d psq(float u, float v, float n, float e);

/* Ray Equation */
Vector3d findRay(Vector3d a, Vector3d b, double t);
/* Apply the Inverse Transform to a to get a new, usable a. */
Vector3d newa(MatrixXd unScale, MatrixXd unRotate, Vector3d a);
/* Apply the Inverse Transform to b to get a new, usable b. */
Vector3d newb(MatrixXd unScale, MatrixXd unRotate, Vector3d unTranslate, Vector3d b);
/* Finds the scalar coefficients of the quadratic equation with the two given
 * vectors. If positiveb is true then the returned coeffs will all be multiplied
 * by -1 if b is negative, to ensure that b is positive. */
Vector3d findCoeffs(Vector3d a, Vector3d b, bool positiveb);
/* Finds the roots of the quadratic with the coefficients specified by the input
 * Vector3d. If one of the roots is complex then FLT_MAX is returned instead. */
Vector2d findRoots(Vector3d coeffs);

/* Uses Newton's method to find the t value at which a ray hits the superquadric.
 * If the ray actually misses the superquadric then FLT_MAX is returned instead.*/
double updateRule(Vector3d a, Vector3d b, double e, double n, double t, double epsilon);
/* Returns -1 for negative numbers, 1 for positive numbers, and 0 for zero. */
int sign(double s);

/* Gradient of the isq function. */
Vector3d isqGradient(Vector3d vec, double e, double n);
/* Derivative of the isq function. */
double gPrime(Vector3d vec, Vector3d a, double e, double n);
/* Unit normal vector at a point on the superquadric */
Vector3d unitNormal(MatrixXd r, Vector3d vec1, Vector3d vec2, double tt, double e, double n);

// These functions exist for use in halfEdge.cpp
/*
 */

// Returns the square of val
double square(double val);

// Returns the distance between a and b. Assumes a and b represent points (or
// the endpoints of the vectors, whatever)
double distance(Tvec *a, Tvec *b);

// Returns true if every value of a is zero
bool vecIsZero(Tvec *a);
