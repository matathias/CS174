
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"

class PhysicalObject : public Object {
    public:
        PhysicalObject(MatrixXd scl, MatrixXd rot, Vector3d pos, Vector3d rgb,
                       float a, float ee, float nn, Vector3d vel, float m);
        void setVelocity(Vector3d vel);
        void setMass(float m);
        Vector3d getVelocity();
        float getMass();
	int get_hitstun();
	int set_hitstun();
	int reduce_hitstun();
    
    protected:
        Vector3d velocity;
        float mass;
	int hitstun_timer;
};
