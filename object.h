
#pragma once

#include <Eigen/Eigen>
 
class Object
{
    public:
        Object(MatrixXd scl, MatrixXd rot, Vector3d pos, float ee, float nn);
        void setPosition(Vector3d pos);
        void setScale(MatrixXd scl);
        void setRotate(MatrixXd rot);
        void setE(float ee);
        void setN(float nn);
        Vector3d getPosition();
        MatrixXd getScale();
        MatrixXd getRotate();
        float getE();
        float getN();
        
    protected:
        // Transforms
        Vector3d position;
        MatrixXd scale;
        MatrixXd rotate;
        
        // These two exponents are for if we're using superquadrics
        float e;
        float n;
}
