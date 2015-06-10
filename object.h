
#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "halfEdge.h"
using namespace Eigen;
using namespace std;
 
class Object
{
    public:
        Object(MatrixXd scl, MatrixXd rot, Vector3d pos, Vector3d rgb, float a,
               float ee, float nn);
        void setPosition(Vector3d pos);
        void setScale(MatrixXd scl);
        void setRotate(MatrixXd rot);
        void setE(float ee);
        void setN(float nn);
        void setRGB(Vector3d rgb);
        void setAlpha(float a);
        Vector3d getPosition();
        MatrixXd getScale();
        MatrixXd getRotate();
        float getE();
        float getN();
        Vector3d getRGB();
        float getAlpha();
        vector<HE*>* getHalfEdges();
        vector<HEF*>* getHalfFaces();
        vector<HEV*>* getHalfVertices();
        vector<Vector3d>* getNormals();
        bool isPhysical();
        bool collidedWith(Object *o);
        Vector3d getFarthestPointInDirection(Vector3d d);
        
    protected:
        // Transforms
        Vector3d position;
        MatrixXd scale;
        MatrixXd rotate;
        
        // These two exponents are for if we're using superquadrics
        float e;
        float n;
        
        // These vectors make up the half-edge data structure that contains the
        // objects faces, vertices, and edges
        vector<HE*> halfEdges;
        vector<HEF*> halfFaces;
        vector<HEV*> halfVertices;
        vector<Vector3d> normals;
        
        // Helps determine if the object is static or not, since c++ doesn't
        // seem to have object-type detection
        bool physical;
        
        // RGB and Alpha values for the object. Used for rendering
        Vector3d RGB;
        float alpha;
        
    private:
        // Method in charge of filling the object's half-edge structure based
        // on the input parameters. This should only ever be called from the
        // object constructor
        void initializeHalfEdge(MatrixXd scl, MatrixXd rot, Vector3d pos);
        // updates all of the object's vertices with the given translation
        // vector. Is only ever called from getPosition()
        void updateVerticesPos(Vector3d trans);
};
