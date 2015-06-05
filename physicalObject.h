
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"

class PhysicalObject : public Object {
    public:
        PhysicalObject(MatrixXd scl, MatrixXd rot, Vector3d pos, float ee, 
                       float nn, Vector3d vel, float m);
        void setVelocity(Vector3d vel);
        void setMass(float m);
        Vector3d getVelocity();
        float getMass();
    
    protected:
        Vector3d velocity;
        float mass;
};
