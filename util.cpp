/* CS/CNS 171 Fall 2014
 *
 * This file contains the actual implementations of the functions in util.h.
 */

#include "util.h"
#include "Object.h"
#include <math.h>
#include <float.h>

/* Gets the translation matrix for a given translation vector (x, y, z). */
MatrixXd get_translate_mat(double x, double y, double z)
{
    MatrixXd translate = MatrixXd::Identity(4,4);
    translate(0,3) = x;
    translate(1,3) = y;
    translate(2,3) = z;
    return translate;
}

/* Gets the rotation matrix for a given rotation axis (x, y, z) and angle. */
MatrixXd get_rotate_mat(double x, double y, double z, double angle)
{
    double nor = sqrt((x * x) + (y * y) + (z * z));
    x = x / nor;
    y = y / nor;
    z = z / nor;
    //angle = angle * M_PI / 180.0;

    MatrixXd rotate = MatrixXd::Identity(4,4);
    rotate(0,0) = pow(x,2) + (1 - pow(x,2)) * cos(angle);
    rotate(0,1) = (x * y * (1 - cos(angle))) - (z * sin(angle));
    rotate(0,2) = (x * z * (1 - cos(angle))) + (y * sin(angle));

    rotate(1,0) = (y * x * (1 - cos(angle))) + (z * sin(angle));
    rotate(1,1) = pow(y,2) + (1 - pow(y,2)) * cos(angle);
    rotate(1,2) = (y * z * (1 - cos(angle))) - (x * sin(angle));

    rotate(2,0) = (z * x * (1 - cos(angle))) - (y * sin(angle));
    rotate(2,1) = (z * y * (1 - cos(angle))) + (x * sin(angle));
    rotate(2,2) = pow(z,2) + (1 - pow(z,2)) * cos(angle);

    return rotate;
}

/* Gets the scaling matrix for a given scaling vector (x, y, z). */
MatrixXd get_scale_mat(double x, double y, double z)
{
    MatrixXd scale = MatrixXd::Identity(4,4);
    scale(0,0) = x;
    scale(1,1) = y;
    scale(2,2) = z;
    return scale;
}

/* Gets the perspective projection matrix given the frustum parameters. */
MatrixXd get_perspective_mat(double near, double far,
                             double left, double right,
                             double top, double bottom)
{
    MatrixXd perspective = MatrixXd::Zero(4,4);
    perspective(0,0) = (2 * near) / (right - left);
    perspective(0,2) = (right + left) / (right - left);
    perspective(1,1) = (2 * near) / (top - bottom);
    perspective(1,2) = (top + bottom) / (top - bottom);
    perspective(2,2) = (-1 * (far + near)) / (far - near);
    perspective(2,3) = (-2 * far * near) / (far - near);
    perspective(3,2) = -1;
    return perspective;
}

/* Takes a Vector3d and turns it into a Vector4d. */
Vector4d vector3to4(Vector3d vec)
{
    Vector4d vec2(vec(0), vec(1), vec(2), 1.0);
    return vec2;
}

/* Takes a Vector4d and turns it into a Vector3d. */
Vector3d vector4to3(Vector4d vec)
{
    Vector3d vec2(vec(0) / (double) vec(3), vec(1) / (double) vec(3), vec(2) / (double) vec(3));
    return vec2;
}

/* Takes a 3x3 MatrixXd and turns it into a 4x4 MatrixXd. */
MatrixXd matrix3to4(MatrixXd mat)
{
    MatrixXd mat2 = MatrixXd::Identity(4,4);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mat2(i,j) = mat(i,j);
        }
    }

    return mat2;
}

/* Takes a 4x4 MatrixXd and turns it into a 3x3 MatrixXd. */
MatrixXd matrix4to3(MatrixXd mat)
{
    MatrixXd mat2 = MatrixXd::Identity(3,3);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            mat2(i,j) = mat(i,j);
        }
    }

    double w = mat(3,3);
    if (w != 0.0)
    {
        mat2 = mat2 / (double) w;
    }

    return mat2;
}

/* Implicit Superquadric function. */
double isq(Vector3d vec, double e, double n)
{
    // Test for n = 0 now to prevent divide-by-zero errors.
    if (n == 0)
        return FLT_MAX;
    
    //double zTerm = pow(pow(vec(2), 2.0), 1.0 / (double) n);
    double zTerm = pow(vec(2), 2.0 / (double) n);

    // Test for e = 0 now to prevent divide-by-zero errors.
    if (e == 0)
        return zTerm;

    //double xTerm = pow(pow(vec(0), 2.0), 1.0 / (double) e);
    //double yTerm = pow(pow(vec(1), 2.0), 1.0 / (double) e);
    double xTerm = pow(vec(0), 2.0 / (double) e);
    double yTerm = pow(vec(1), 2.0 / (double) e);
    double xyTerm = pow(xTerm + yTerm, e / (double) n);

    return xyTerm + zTerm - 1.0;
}

/* Parametric cosine analog function. */
double cosine(double u, double e)
{
    double cosU = cos(u);
    double sign = 1.0;

    if (cosU == 0)
        return 0;
    else if (cosU < 0)
        sign = -1.0;

    cosU = pow(abs(cosU), e);
    return sign * cosU;
}

/* Parametric sine analog function. */
double sine(double u, double e)
{
    double sinU = sin(u);
    double sign = 1.0;

    if (sinU == 0)
        return 0;
    else if (sinU < 0)
        sign = -1.0;

    sinU = pow(abs(sinU), e);
    return sign * sinU;
}

/* Parametric superquadric function. */
Vector3d psq(float u, float v, float n, float e)
{
    float x = cosine(v,n) * cosine(u,e);
    float y = cosine(v,n) * sine(u,e);
    float z = sine(v,n);
    Vector3d vec(x, y, z);
    return vec;
}

/* Ray Equation */
Vector3d findRay(Vector3d a, Vector3d b, double t)
{
    return (a * t) + b;
}

/* Apply the Inverse Transform to a to get a new, usable a. */
Vector3d newa(MatrixXd unScale, MatrixXd unRotate, Vector3d a)
{
    return unScale * (unRotate * a);
}

/* Apply the Inverse Transform to b to get a new, usable b. */
Vector3d newb(MatrixXd unScale, MatrixXd unRotate, Vector3d unTranslate, Vector3d b)
{
    return unScale * (unRotate * (b + unTranslate));
}

/* Finds the scalar coefficients of the quadratic equation with the two given
 * vectors. If positiveb is true then the returned coeffs will all be multiplied
 * by -1 if be is negative, to ensure that b is positive. */
Vector3d findCoeffs(Vector3d a, Vector3d b, bool positiveb)
{
    Vector3d coeffs(a.dot(a), 2 * a.dot(b), b.dot(b) - 3);
    if (positiveb)
    {
        if (coeffs(1) < 0)
            coeffs = coeffs * -1;
    }
    return coeffs;
}

/* Finds the roots of the quadratic with the coefficients specified by the input
 * Vector3d. If one of the roots is complex then FLT_MAX is returned instead. */
Vector2d findRoots(Vector3d coeffs)
{
    double tMinus = 0.0, tPlus = 0.0;
    double interior = pow(coeffs(1), 2) - (4 * coeffs(0) * coeffs(2));
    if (interior < 0)
    {
        tMinus = tPlus = FLT_MAX;
    }
    else
    {
        tMinus = (-coeffs(1) - sqrt(interior)) / (double) (2 * coeffs(0));
        tPlus = (2 * coeffs(2)) / (double) (-coeffs(1) - sqrt(interior));
    }
    Vector2d roots(tMinus, tPlus);
    return roots;
}

/* Uses Newton's method to find the t value at which a ray hits the superquadric.
 * If the ray actually misses the superquadric then FLT_MAX is returned instead.*/
double updateRule(Vector3d a, Vector3d b, double e, double n, double t, double epsilon)
{
    Vector3d vec = findRay(a, b, t);
    double gP = gPrime(vec, a, e, n);
    double gPPrevious = gP;
    double g = 0.0;
    double tnew = t, told = t;
    bool stopPoint = false;

    while (!stopPoint)
    {
        told = tnew;
        vec = findRay(a, b, told);
        gP = gPrime(vec, a, e, n);
        g = isq(vec, e, n);

        if ((g - epsilon) <= 0)
        {
            stopPoint = true;
        }
        else if (sign(gP) != sign(gPPrevious) || (gP == 0 && (g - epsilon) > 0))
        {
            stopPoint = true;
            tnew = FLT_MAX;
        }
        else
        {
            tnew = told - (g / gP);
            gPPrevious = gP;
        }
    }
    return tnew;
}

/* Returns -1 for negative numbers, 1 for positive numbers, and 0 for zero. */
int sign(double s)
{
    if(s > 0) return 1;
    if(s < 0) return -1;
    return 0;
}

/* Gradient of the isq function. */
Vector3d isqGradient(Vector3d vec, double e, double n)
{
    double xval = 0.0, yval = 0.0, zval = 0.0;
    // Check for n = 0 to prevent divide-by-zero errors
    if (n == 0)
    {
        cout << "n is zero!" << endl;
        xval = yval = zval = FLT_MAX;
    }
    // Check for e = 0 to prevent divide-by-zero errors
    else if (e == 0)
    {
        cout << "e is zero!" << endl;
        xval = yval = FLT_MAX;
        zval = (2 * vec(2) * pow(pow(vec(2), 2), ((double) 1 / n) - 1)) / (double) n;
    }
    else
    {
        double xterm = pow(vec(0), (double) 2 / e);
        double yterm = pow(vec(1), (double) 2 / e);
        double xyterm = pow(xterm + yterm, ((double) e / n) - 1);
        double x2term = (2 * pow(vec(0), ((double) 2 / e) - 1));
        double y2term = (2 * pow(vec(1), ((double) 2 / e) - 1));
        xval = x2term * xyterm / (double) n;
        yval = y2term * xyterm / (double) n;
        zval = (2 * pow(vec(2), ((double) 2 / n) - 1)) / (double) n;
    }
    
    Vector3d res(xval, yval, zval);
    return res;
}

/* Derivative of the isq function. */
double gPrime(Vector3d vec, Vector3d a, double e, double n)
{
    return a.dot(isqGradient(vec, e, n));
}

/* Unit normal vector at a point on the superquadric */
Vector3d unitNormal(MatrixXd r, Vector3d vec1, Vector3d vec2, double tt, double e, double n)
{
    //Vector3d nor = newb(s, r, t, isqGradient(vec, e, n));
    Vector3d nor = r * isqGradient(findRay(vec1, vec2, tt), e, n);
    nor.normalize();
    return nor;
}

/* These functions exist primarily for use in the collision detection algorithm.
 */
Vector3d tripleProduct(Vector3d a, Vector3d b, Vector3d c)
{
    return b * (c.dot(a)) - a * (c.dot(b));
}

Vector3d support(Object *a, Object *b, Vector3d d)
{
    Vector3d p1 = a->getFarthestPointInDirection(d);
    Vector3d p2 = b->getFarthestPointInDirection(d);
    
    return p1 - p2;
}

// Returns true if the given simplex s contains the origin, and false otherwise
bool containsOrigin(vector<Vector3d> *s, Vector3d *d)
{
    // Get the last point added to the simplex
    Vector3d a = s->back();
    
    Vector3d ao = a * -1;
    
    if (s->size() == 4) { // tetrahedron case (a at 3, b at 2, c at 1, d at 0)
        // get edges AB and AC from triangle ABC
        Vector3d ab = s->at(2) - a; // b - a
        Vector3d ac = s->at(1) - a; // c - a
        // Get the face normal
        Vector3d abcNormal = ab.cross(ac);
        // If the origin is in the region bordering triangle ABC
        if (abcNormal.dot(ao) > 0) {
            // Remove point d
            s->erase(0);
            // set the new direction to abcNormal
            *d = abcNormal;
        }
        else {
            // get edges DB and DA from triangle ABD
            Vector3d db = s->at(2) - s->at(0); // b - d
            Vector3d da = a - s->at(0); // a - d
            // Get the face normal
            Vector3d abdNormal = db.cross(da);
            // If the origin is in the region bordering triangle ABD
            if (abdNormal.dot(ao) > 0) {
                // Remove point c
                s->erase(1);
                // set the new direction to abdNormal
                *d = abdNormal;
            }
            else {
                // get edge DC from triangle ACD (we already have da)
                Vector3d dc = s->at(1) - s->at(0); // c - d
                // Get the face normal
                Vector3d acdNormal = da.cross(dc);
                // If the origin is in the region bordering triangle ACD
                if (acdNormal.dot(ao) > 0) {
                    // remove point b
                    s->erase(2);
                    // set the new direction to acdNormal
                    *d = acdNormal;
                }
                else {
                    // We now know that the origin is inside the simplex
                    return true;
                }
            }
        }
    
    }
    else if (s->size() == 3) { // triangle case (a at 2, b at 1, c at 0)
        // Get the normal vector to the triangle
        Vector3d ab = s->at(1) - a; // b - a
        Vector3d ac = s->at(0) - a; // c - a
        Vector3d normal = ab.cross(ac);
        // See if the normal is in the direction of the origin. If it isn't,
        // negate it so that it is
        if (normal.dot(ao) < 0) {
            normal = normal * -1;
        }
        // set this vector as the new direction
        *d = normal;
    }
    else { // line segment case (a at 1, b at 0)
        Vector3d b = s->at(0);
        Vector3d ab = b - a;
        // Set the direction to perpendicular to ab in the direction of the
        // origin
        *d = tripleProduct(ab, ao, ab);
    }
    
    return false;
}

// These functions exist for use in halfEdge.cpp
/*
 */

// Returns the square of val
double square(double val)
{
    return val * val;
}

// Returns the distance between a and b. Assumes a and b represent points (or
// the endpoints of the vectors, whatever)
double distance(Vector3d a, Vector3d b)
{
    return sqrt(square(b(0) - a(0)) + square(b(1) - a(1)) + square(b(2) - a(2)));
}

// Returns true if every value of a is zero
bool vecIsZero(Vector3d a)
{
    return (a(0) == 0.0 && a(1) == 0.0 && a(2) == 0.0);
}
