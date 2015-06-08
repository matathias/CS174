
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#include "object.h"

enum boundary_t {
    GROUND,
    WALL_LEFT,
    WALL_RIGHT,
    WALL_FRONT,
    WALL_BACK,
    CEILING
};

class BoundaryObject : public Object {
    public:
        BoundaryObject(boundary_t b);
        void setBoundaryType(boundary_t b);
        void setDampening(float d);
        boundary_t getBoundaryType();
        float getDampening();
    
    protected:
        boundary_t bot;
        float dampening;
};