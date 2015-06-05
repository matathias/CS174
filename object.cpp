/* Class file for objects.
 * 
 * This file contains the implementation for a basic object. Each object stores
 * its own position, velocity, and mass; more advanced objects can be
 * implemented as subclasses with this as the superclass.
 *
 * Something something OOP.
 */
 
class Object
{
    public:
        Object(float *pos, float *vel, float m);
        void setPosition(float *pos);
        void setVelocity(float *vel);
        void setMass(float m);
        float* getPosition();
        float* getVelocity();
        float getMass();

    private:
        float position[3];
        float velocity[3];
        float mass;
}
 
/********** Member function definitions **********/
// Constructor
Object::Object(float *pos, float *vel, float m)
{
    for (int i = 0; i < 3; i++) {
        position[i] = pos[i];
        velocity[i] = vel[i];
    }
    
    mass = m;
}

// Manipulators
void Object::setPosition(float *pos)
{
    position[0] = pos[0];
    position[1] = pos[1];
    position[2] = pos[2];
}

void Object::setVelocity(float *vel)
{
    velocity[0] = vel[0];
    velocity[1] = vel[1];
    velocity[2] = vel[2];
}

void Object::setMass(float m)
{
    mass = m;
}

// "Get" functions
