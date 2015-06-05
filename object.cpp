/* Class file for objects.
 * 
 * This file contains the implementation for a basic object. Each object stores
 * its own position, velocity, and mass; more advanced objects can be
 * implemented as subclasses with this as the superclass.
 *
 * Something something OOP.
 */
 
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
 
class Object
{
    public:
        Object(Vector3d pos, Vector3d vel, float m);
        void setPosition(Vector3d pos);
        void setVelocity(Vector3d vel);
        void setMass(float m);
        Vector3d getPosition();
        Vector3d getVelocity();
        float getMass();

    private:
        Vector3d position;
        Vector3d velocity;
        float mass;
}
 
/********** Member function definitions **********/
// Constructor
Object::Object(Vector3d pos, Vector3d vel, float m)
{
    position = pos;
    velocity = vel;
    
    mass = m;
}

// Manipulators
void Object::setPosition(Vector3d pos)
{
    position = pos;
}

void Object::setVelocity(Vector3d vel)
{
    velocity = vel;
}

void Object::setMass(float m)
{
    mass = m;
}

// "Get" functions
Vector3d Object::getPosition()
{
    return position;
}

Vector3d Object::getVelocity()
{
    return velocity;
}

float Object::getMass()
{
    return mass;
}
